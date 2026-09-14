#include "packet_comm.h"
#include "bsp_buzzer.h"
#include "bsp_fan.h"
#include "bsp_gas_valve.h"
#include "bsp_limit_switch.h"
#include "bsp_plasma_feedback.h"
#include "bsp_relay.h"
#include "bsp_rgb.h"
#include "gas_pressure.h"
#include "mfc.h"
#include "modbus_rtu.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define LED_ON GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET

#define RELAY_PLASMA_ON 0x10U
#define RELAY_VOL_MOD_ON 0x01U
#define CTR_ASS_FAN_ON 0x20U
#define DEVICE_FAN_ON 0x40U
#define CTR_MAIN_FAN_ON 0x80U
#define RELAY_FLOW_METER_ON 0x10U
#define RELAY_GAS_VALVE_ON 0x01U
#define BUZZER_1_ON 0x02U
#define BUZZER_2_ON 0x04U
#define RGB_RED_ON 0x02U
#define RGB_GREEN_ON 0x04U
#define RGB_YELLOW_ON 0x08U
#define PACKET_COMM_FAN_DUTY_COMMAND 0xA5U
#define PACKET_COMM_UART_BAUDRATE 115200U
#define PACKET_COMM_UART_TX_MARGIN_MS 200U
#define HARDWARE_EMERGENCY_BLINK_MS 150U
#define HARDWARE_EMERGENCY_BEEP_COUNT 5U
#define HARDWARE_EMERGENCY_RESTORE_BEEP_COUNT 2U
#define PACKET_COMM_CONNECT_REQ_EMER 0xC5U
#define PACKET_COMM_CONNECT_ACK_EMER 0xCAU
#define PACKET_COMM_CONNECT_VOL_RELAY 0xA5U
#define PACKET_COMM_CONNECT_VOL_VALUE 0x5AA5U
#define PACKET_COMM_CONNECT_HE_RELAY 0x3CU
#define PACKET_COMM_CONNECT_HE_VALUE 0xC33CU
#define PACKET_COMM_CONNECT_AR_RELAY 0x96U
#define PACKET_COMM_CONNECT_AR_VALUE 0x6996U
#define PACKET_COMM_YELLOW_SLOW_BLINK_MS 1000U
#define PACKET_COMM_LED_RUN_MS 200U
#define PACKET_COMM_LED_EMERGENCY_BLINK_MS 250U

volatile bool PacketComm_PackReady = false;
PacketComm_DataPacket PacketComm_RecvPacket = {0};
PacketComm_DataPacket PacketComm_SendPacket = {0};

typedef struct
{
	UART_HandleTypeDef *uart;
	uint8_t rxByte;
	uint8_t rxBuf[PACKET_COMM_RX_BUF_SIZE];
	volatile uint16_t rxLen;
} PacketComm_RxContext;

static PacketComm_RxContext packet_ports[PACKET_COMM_PORT_COUNT] = {0};
static uint8_t statusEmerStop = 0U;
static uint16_t statusVolOutValue = 0U;
static bool hardwareEmergencyLatched = false;
static uint32_t hardwareEmergencyBlinkTick = 0U;
static bool hardwareEmergencyRedOn = false;
static uint32_t connectionIndicatorTick = 0U;
static bool connectionYellowOn = false;
static uint32_t panelLedTick = 0U;
static uint8_t panelLedIndex = 0U;
static bool panelLedBlinkOn = false;
static bool cmdUartConnected = false;

static float PacketComm_CentiLpmToFlow(uint16_t value)
{
	return (float)value / 100.0f;
}

static uint16_t PacketComm_FlowToCentiLpm(float value)
{
	if (value <= 0.0f)
	{
		return 0U;
	}

	if (value >= 655.35f)
	{
		return 65535U;
	}

	return (uint16_t)((value * 100.0f) + 0.5f);
}

static uint8_t PacketComm_DutyToPercent(uint16_t duty)
{
	if (duty >= 1000U)
	{
		return 100U;
	}

	return (uint8_t)((duty + 5U) / 10U);
}

static int32_t PacketComm_FloatToCenti(float value)
{
	if (value >= 0.0f)
	{
		return (int32_t)((value * 100.0f) + 0.5f);
	}

	return (int32_t)((value * 100.0f) - 0.5f);
}

static int32_t PacketComm_FloatToDeci(float value)
{
	if (value >= 0.0f)
	{
		return (int32_t)((value * 10.0f) + 0.5f);
	}

	return (int32_t)((value * 10.0f) - 0.5f);
}

static uint32_t PacketComm_Abs32(int32_t value)
{
	return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static const char *PacketComm_OnOff(bool on)
{
	return on ? "ON" : "OFF";
}

static const char *PacketComm_PressedReleased(bool pressed)
{
	return pressed ? "PRESSED" : "RELEASED";
}

static const char *PacketComm_ValidFault(bool valid)
{
	return valid ? "OK" : "FAULT";
}

static const char *PacketComm_ConfigText(bool configured)
{
	return configured ? "CONFIGURED" : "NOT_CONFIGURED";
}

static const char *PacketComm_MfcPowerText(mfc_channel_t channel)
{
	if (!MFC_IsChannelEnabled(channel))
	{
		return "DISABLED";
	}

	switch (MFC_GetPowerState(channel))
	{
	case MFC_POWER_OFF:
		return "OFF";

	case MFC_POWER_STARTING:
		return "STARTING";

	case MFC_POWER_ON:
		return "ON";

	case MFC_POWER_STOPPING:
		return "STOPPING";

	default:
		return "FAULT";
	}
}

static const char *PacketComm_MfcTrackingText(mfc_channel_t channel)
{
	if (!MFC_IsChannelEnabled(channel))
	{
		return "DISABLED";
	}

	if (!MFC_IsPowered(channel))
	{
		return "OFF";
	}

	if (!MFC_IsFeedbackValid(channel))
	{
		return "WAIT";
	}

	return MFC_IsTrackingNormal(channel) ? "OK" : "FAULT";
}

static uint32_t PacketComm_TxTimeoutMs(uint16_t length)
{
	uint32_t timeout = ((uint32_t)length * 10U * 1000U) / PACKET_COMM_UART_BAUDRATE;

	timeout += PACKET_COMM_UART_TX_MARGIN_MS;
	if (timeout < 300U)
	{
		timeout = 300U;
	}

	return timeout;
}

static uint16_t ReadU16LE(const uint8_t *data)
{
	return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t ReadU32LE(const uint8_t *data)
{
	return (uint32_t)data[0] |
		   ((uint32_t)data[1] << 8) |
		   ((uint32_t)data[2] << 16) |
		   ((uint32_t)data[3] << 24);
}

static void WriteU16LE(uint8_t *data, uint16_t value)
{
	data[0] = (uint8_t)(value & 0xFFU);
	data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void WriteU32LE(uint8_t *data, uint32_t value)
{
	data[0] = (uint8_t)(value & 0xFFU);
	data[1] = (uint8_t)((value >> 8) & 0xFFU);
	data[2] = (uint8_t)((value >> 16) & 0xFFU);
	data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void WriteFloatLE(uint8_t *data, float value)
{
	union
	{
		float f32;
		uint32_t u32;
	} raw;

	raw.f32 = value;
	WriteU32LE(data, raw.u32);
}

static void DropRxBytes(PacketComm_RxContext *ctx, uint16_t count)
{
	uint16_t len = ctx->rxLen;

	if (count >= len)
	{
		ctx->rxLen = 0;
		return;
	}

	memmove(ctx->rxBuf, &ctx->rxBuf[count], len - count);
	ctx->rxLen = len - count;
}

static uint16_t PacketComm_FanDutyFromCommand(bool enabled, uint16_t commandDuty)
{
	if (!enabled)
	{
		return 0U;
	}

	if (commandDuty == 0U)
	{
		return 1000U;
	}

	if (commandDuty > 1000U)
	{
		return 1000U;
	}

	return commandDuty;
}

static void PacketComm_SetPanelLeds(bool led1, bool led2, bool led3)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, led1 ? LED_ON : LED_OFF);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, led2 ? LED_ON : LED_OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, led3 ? LED_ON : LED_OFF);
}

static PacketComm_RxContext *PacketComm_GetContext(PacketComm_Port port)
{
	if (port >= PACKET_COMM_PORT_COUNT)
	{
		return NULL;
	}

	return &packet_ports[port];
}

static PacketComm_RxContext *PacketComm_FindContext(UART_HandleTypeDef *huart)
{
	for (uint8_t i = 0U; i < PACKET_COMM_PORT_COUNT; i++)
	{
		if (packet_ports[i].uart == huart)
		{
			return &packet_ports[i];
		}
	}

	return NULL;
}

void PacketComm_Init(UART_HandleTypeDef *logUart, UART_HandleTypeDef *hostUart)
{
	packet_ports[PACKET_COMM_PORT_LOG].uart = logUart;
	packet_ports[PACKET_COMM_PORT_HOST].uart = hostUart;

	for (uint8_t i = 0U; i < PACKET_COMM_PORT_COUNT; i++)
	{
		packet_ports[i].rxLen = 0U;
	}
	PacketComm_PackReady = false;
	cmdUartConnected = false;

	PacketComm_SendPacket.header = PACKET_COMM_HEADER;
	if (logUart != NULL)
	{
		HAL_UART_Receive_IT(logUart, &packet_ports[PACKET_COMM_PORT_LOG].rxByte, 1);
	}
	if (hostUart != NULL)
	{
		HAL_UART_Receive_IT(hostUart, &packet_ports[PACKET_COMM_PORT_HOST].rxByte, 1);
	}
}

uint32_t PacketComm_CalculateChecksum(const uint8_t *packet)
{
	uint32_t checksum = 0;

	for (uint8_t i = 0; i < 12U; i++)
	{
		checksum += packet[i];
	}

	return checksum;
}

static bool PacketComm_IsConnectFrame(const uint8_t *packet, uint8_t emerValue)
{
	return (packet[2] == emerValue) &&
		   (packet[3] == PACKET_COMM_CONNECT_VOL_RELAY) &&
		   (ReadU16LE(&packet[4]) == PACKET_COMM_CONNECT_VOL_VALUE) &&
		   (packet[6] == PACKET_COMM_CONNECT_HE_RELAY) &&
		   (ReadU16LE(&packet[7]) == PACKET_COMM_CONNECT_HE_VALUE) &&
		   (packet[9] == PACKET_COMM_CONNECT_AR_RELAY) &&
		   (ReadU16LE(&packet[10]) == PACKET_COMM_CONNECT_AR_VALUE);
}

static HAL_StatusTypeDef PacketComm_SendConnectAck(PacketComm_RxContext *ctx)
{
	uint8_t packet[PACKET_COMM_SIZE] = {0};

	if ((ctx == NULL) || (ctx->uart == NULL))
	{
		return HAL_ERROR;
	}

	WriteU16LE(&packet[0], PACKET_COMM_HEADER);
	packet[2] = PACKET_COMM_CONNECT_ACK_EMER;
	packet[3] = PACKET_COMM_CONNECT_VOL_RELAY;
	WriteU16LE(&packet[4], PACKET_COMM_CONNECT_VOL_VALUE);
	packet[6] = PACKET_COMM_CONNECT_HE_RELAY;
	WriteU16LE(&packet[7], PACKET_COMM_CONNECT_HE_VALUE);
	packet[9] = PACKET_COMM_CONNECT_AR_RELAY;
	WriteU16LE(&packet[10], PACKET_COMM_CONNECT_AR_VALUE);
	WriteU32LE(&packet[12], PacketComm_CalculateChecksum(packet));

	return HAL_UART_Transmit(ctx->uart,
							 packet,
							 PACKET_COMM_SIZE,
							 PacketComm_TxTimeoutMs(PACKET_COMM_SIZE));
}

PacketComm_Result PacketComm_Process(PacketComm_Port port)
{
	uint8_t packet[PACKET_COMM_SIZE];
	uint16_t len;
	PacketComm_RxContext *ctx = PacketComm_GetContext(port);

	if ((ctx == NULL) || (ctx->uart == NULL))
	{
		return PACKET_COMM_NO_PACKET;
	}

	__disable_irq();
	len = ctx->rxLen;
	__enable_irq();

	while (len >= 2U)
	{
		if (ReadU16LE(ctx->rxBuf) == PACKET_COMM_HEADER)
		{
			break;
		}

		__disable_irq();
		DropRxBytes(ctx, 1U);
		len = ctx->rxLen;
		__enable_irq();
	}

	if (len < PACKET_COMM_SIZE)
	{
		return PACKET_COMM_NO_PACKET;
	}

	__disable_irq();
	memcpy(packet, ctx->rxBuf, PACKET_COMM_SIZE);
	DropRxBytes(ctx, PACKET_COMM_SIZE);
	__enable_irq();

	if (PacketComm_CalculateChecksum(packet) != ReadU32LE(&packet[12]))
	{
		return PACKET_COMM_BAD_CHECKSUM;
	}

	if ((port == PACKET_COMM_PORT_HOST) &&
		PacketComm_IsConnectFrame(packet, PACKET_COMM_CONNECT_REQ_EMER))
	{
		cmdUartConnected = true;
		(void)PacketComm_SendConnectAck(ctx);
		PacketComm_PackReady = false;
		return PACKET_COMM_NO_PACKET;
	}

	PacketComm_RecvPacket.header = ReadU16LE(&packet[0]);
	PacketComm_RecvPacket.EmerStop = packet[2];
	PacketComm_RecvPacket.VolRelay = packet[3];
	PacketComm_RecvPacket.VolOutValue = ReadU16LE(&packet[4]);
	PacketComm_RecvPacket.HeFLOWRelay = packet[6];
	PacketComm_RecvPacket.HeOutValue = ReadU16LE(&packet[7]);
	PacketComm_RecvPacket.ArFLOWRelay = packet[9];
	PacketComm_RecvPacket.ArOutValue = ReadU16LE(&packet[10]);
	PacketComm_RecvPacket.footer = ReadU32LE(&packet[12]);
	PacketComm_PackReady = true;

	return PACKET_COMM_OK;
}

static void PacketComm_UpdateStatePacket(void)
{
	uint8_t volRelay = 0U;
	uint8_t heFlowRelay = 0U;
	uint8_t arFlowRelay = 0U;

	if (BoardRelay_Get(plasma_relay))
	{
		volRelay |= RELAY_PLASMA_ON;
	}
	if (BoardRelay_Get(vol_mod_relay))
	{
		volRelay |= RELAY_VOL_MOD_ON;
	}
	if (BoardFan_Get(ctr_ass_fan))
	{
		volRelay |= CTR_ASS_FAN_ON;
	}
	if (BoardFan_Get(device_fan))
	{
		volRelay |= DEVICE_FAN_ON;
	}
	if (BoardFan_Get(ctr_main_fan))
	{
		volRelay |= CTR_MAIN_FAN_ON;
	}

	if (MFC_IsPowered(He_MFC))
	{
		heFlowRelay |= RELAY_FLOW_METER_ON;
	}
	if (BoardGasValve_Get(he_valve))
	{
		heFlowRelay |= RELAY_GAS_VALVE_ON;
	}
	if (BoardBuzzer_Get(buzzer_1))
	{
		heFlowRelay |= BUZZER_1_ON;
	}
	if (BoardBuzzer_Get(buzzer_2))
	{
		heFlowRelay |= BUZZER_2_ON;
	}

	if (MFC_IsPowered(Ar_MFC))
	{
		arFlowRelay |= RELAY_FLOW_METER_ON;
	}
	if (BoardGasValve_Get(ar_valve))
	{
		arFlowRelay |= RELAY_GAS_VALVE_ON;
	}
	if (BoardRgb_Get(rgb_red))
	{
		arFlowRelay |= RGB_RED_ON;
	}
	if (BoardRgb_Get(rgb_green))
	{
		arFlowRelay |= RGB_GREEN_ON;
	}
	if (BoardRgb_Get(rgb_yellow))
	{
		arFlowRelay |= RGB_YELLOW_ON;
	}

	PacketComm_SendPacket.header = PACKET_COMM_HEADER;
	PacketComm_SendPacket.EmerStop = statusEmerStop;
	PacketComm_SendPacket.VolRelay = volRelay;
	PacketComm_SendPacket.VolOutValue = statusVolOutValue;
	PacketComm_SendPacket.HeFLOWRelay = heFlowRelay;
	PacketComm_SendPacket.HeOutValue = PacketComm_FlowToCentiLpm(MFC_GetTargetFlow(He_MFC));
	PacketComm_SendPacket.ArFLOWRelay = arFlowRelay;
	PacketComm_SendPacket.ArOutValue = PacketComm_FlowToCentiLpm(MFC_GetTargetFlow(Ar_MFC));
	PacketComm_SendPacket.footer = 0U;
}

static void PacketComm_PackState(uint8_t *packet)
{
	WriteU16LE(&packet[0], PacketComm_SendPacket.header);
	packet[2] = PacketComm_SendPacket.EmerStop;
	packet[3] = PacketComm_SendPacket.VolRelay;
	WriteU16LE(&packet[4], PacketComm_SendPacket.VolOutValue);
	packet[6] = PacketComm_SendPacket.HeFLOWRelay;
	WriteU16LE(&packet[7], PacketComm_SendPacket.HeOutValue);
	packet[9] = PacketComm_SendPacket.ArFLOWRelay;
	WriteU16LE(&packet[10], PacketComm_SendPacket.ArOutValue);
	WriteU32LE(&packet[12], PacketComm_CalculateChecksum(packet));
	PacketComm_SendPacket.footer = ReadU32LE(&packet[12]);
}

static void PacketComm_ShutdownOutputs(bool keepErrorBuzzer)
{
	BoardRelay_Set(plasma_relay, false);
	BoardRelay_Set(vol_mod_relay, false);
	if (MFC_IsPowered(He_MFC))
	{
		(void)MFC_PowerOff(He_MFC);
	}
	if (MFC_IsPowered(Ar_MFC))
	{
		(void)MFC_PowerOff(Ar_MFC);
	}
	BoardGasValve_Set(he_valve, false);
	BoardGasValve_Set(ar_valve, false);
	BoardFan_Set(ctr_ass_fan, false);
	BoardFan_Set(device_fan, false);
	BoardFan_Set(ctr_main_fan, false);
	if (!keepErrorBuzzer)
	{
		BoardBuzzer_Set(buzzer_1, false);
	}
	BoardBuzzer_Set(buzzer_2, false);
	BoardRgb_AllOff();

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, LED_OFF);
}

static float PacketComm_GetPlasmaFeedbackVpp(void)
{
	plasma_feedback_measurement_t measurement = {0};

	if (!PlasmaFeedback_GetMeasurement(&measurement))
	{
		return 0.0f;
	}

	return measurement.feedback_vpp_v;
}

static float PacketComm_GetGasPressureValue(Gas_Channel_t channel)
{
	GasPressure_Data_t data = {0};

	if (!GasPressure_GetData(channel, &data) || !data.valid)
	{
		return 0.0f;
	}

	return data.pressure;
}

static void PacketComm_ApplyHardwareEmergencyOutputs(void)
{
	BoardRelay_Set(plasma_relay, false);
	BoardRelay_Set(vol_mod_relay, false);
	BoardGasValve_Set(he_valve, false);
	BoardGasValve_Set(ar_valve, false);
	(void)MFC_SetFlow(He_MFC, 0.0f);
	(void)MFC_SetFlow(Ar_MFC, 0.0f);
	BoardBuzzer_Set(buzzer_2, false);
}

void PacketComm_ApplyReceivedPacket(void)
{
	if (!PacketComm_PackReady)
	{
		return;
	}

	if (BoardLimitSwitch_IsPressed(emergency_stop_switch))
	{
		PacketComm_PackReady = false;
		return;
	}

	if (PacketComm_RecvPacket.EmerStop == PACKET_COMM_FAN_DUTY_COMMAND)
	{
		BoardFan_SetDuty(ctr_ass_fan,
						 PacketComm_FanDutyFromCommand((PacketComm_RecvPacket.VolRelay & CTR_ASS_FAN_ON) != 0U,
													   PacketComm_RecvPacket.VolOutValue));
		BoardFan_SetDuty(device_fan,
						 PacketComm_FanDutyFromCommand((PacketComm_RecvPacket.VolRelay & DEVICE_FAN_ON) != 0U,
													   PacketComm_RecvPacket.HeOutValue));
		BoardFan_SetDuty(ctr_main_fan,
						 PacketComm_FanDutyFromCommand((PacketComm_RecvPacket.VolRelay & CTR_MAIN_FAN_ON) != 0U,
													   PacketComm_RecvPacket.ArOutValue));
		statusEmerStop = 0U;
	}
	else if (PacketComm_RecvPacket.EmerStop != 0U)
	{
		PacketComm_ShutdownOutputs(false);
		statusEmerStop = PacketComm_RecvPacket.EmerStop;
	}
	else
	{
		BoardRelay_Set(plasma_relay, (PacketComm_RecvPacket.VolRelay & RELAY_PLASMA_ON) != 0U);
		BoardRelay_Set(vol_mod_relay, (PacketComm_RecvPacket.VolRelay & RELAY_VOL_MOD_ON) != 0U);
		BoardFan_Set(ctr_ass_fan, (PacketComm_RecvPacket.VolRelay & CTR_ASS_FAN_ON) != 0U);
		BoardFan_Set(device_fan, (PacketComm_RecvPacket.VolRelay & DEVICE_FAN_ON) != 0U);
		BoardFan_Set(ctr_main_fan, (PacketComm_RecvPacket.VolRelay & CTR_MAIN_FAN_ON) != 0U);
		if ((PacketComm_RecvPacket.HeFLOWRelay & RELAY_FLOW_METER_ON) != 0U)
		{
			if (!MFC_IsPowered(He_MFC))
			{
				(void)MFC_PowerOn(He_MFC);
			}
			if (MFC_IsPowered(He_MFC))
			{
				(void)MFC_SetFlow(He_MFC, PacketComm_CentiLpmToFlow(PacketComm_RecvPacket.HeOutValue));
			}
		}
		else if (MFC_IsPowered(He_MFC))
		{
			(void)MFC_PowerOff(He_MFC);
		}

		if ((PacketComm_RecvPacket.ArFLOWRelay & RELAY_FLOW_METER_ON) != 0U)
		{
			if (!MFC_IsPowered(Ar_MFC))
			{
				(void)MFC_PowerOn(Ar_MFC);
			}
			if (MFC_IsPowered(Ar_MFC))
			{
				(void)MFC_SetFlow(Ar_MFC, PacketComm_CentiLpmToFlow(PacketComm_RecvPacket.ArOutValue));
			}
		}
		else if (MFC_IsPowered(Ar_MFC))
		{
			(void)MFC_PowerOff(Ar_MFC);
		}
		BoardGasValve_Set(he_valve, (PacketComm_RecvPacket.HeFLOWRelay & RELAY_GAS_VALVE_ON) != 0U);
		BoardGasValve_Set(ar_valve, (PacketComm_RecvPacket.ArFLOWRelay & RELAY_GAS_VALVE_ON) != 0U);
		BoardBuzzer_Start(buzzer_1,
						  ((PacketComm_RecvPacket.HeFLOWRelay & BUZZER_1_ON) != 0U) ?
							  BUZZER_MODE_LONG :
							  BUZZER_MODE_OFF);
		BoardBuzzer_Set(buzzer_2, (PacketComm_RecvPacket.HeFLOWRelay & BUZZER_2_ON) != 0U);
		BoardRgb_Set(rgb_red, (PacketComm_RecvPacket.ArFLOWRelay & RGB_RED_ON) != 0U);
		BoardRgb_Set(rgb_green, (PacketComm_RecvPacket.ArFLOWRelay & RGB_GREEN_ON) != 0U);
		BoardRgb_Set(rgb_yellow, (PacketComm_RecvPacket.ArFLOWRelay & RGB_YELLOW_ON) != 0U);

		statusEmerStop = 0U;
		statusVolOutValue = PacketComm_RecvPacket.VolOutValue;
	}

	PacketComm_PackReady = false;
}

void PacketComm_ProcessHardwareEmergency(void)
{
	bool active = BoardLimitSwitch_IsPressed(emergency_stop_switch);
	uint32_t now = HAL_GetTick();

	if (active)
	{
		if (!hardwareEmergencyLatched)
		{
			BoardBuzzer_StartRepeat(buzzer_1, HARDWARE_EMERGENCY_BEEP_COUNT);
			hardwareEmergencyBlinkTick = now;
			hardwareEmergencyRedOn = true;
			BoardRgb_Set(rgb_green, false);
			BoardRgb_Set(rgb_yellow, false);
			BoardRgb_Set(rgb_red, true);
			hardwareEmergencyLatched = true;
		}

		PacketComm_ApplyHardwareEmergencyOutputs();
		statusEmerStop = 1U;

		if ((now - hardwareEmergencyBlinkTick) >= HARDWARE_EMERGENCY_BLINK_MS)
		{
			hardwareEmergencyBlinkTick = now;
			hardwareEmergencyRedOn = !hardwareEmergencyRedOn;
			BoardRgb_Set(rgb_red, hardwareEmergencyRedOn);
			BoardRgb_Set(rgb_green, false);
			BoardRgb_Set(rgb_yellow, false);
		}
	}
	else
	{
		if (hardwareEmergencyLatched)
		{
			hardwareEmergencyLatched = false;
			hardwareEmergencyRedOn = false;
			BoardBuzzer_Start(buzzer_1, BUZZER_MODE_OFF);
			BoardBuzzer_StartRepeat(buzzer_2, HARDWARE_EMERGENCY_RESTORE_BEEP_COUNT);
			BoardRgb_Set(rgb_red, false);
			BoardRgb_Set(rgb_yellow, false);
			BoardRgb_Set(rgb_green, true);
			if (statusEmerStop == 1U)
			{
				statusEmerStop = 0U;
			}
		}
	}
}

void PacketComm_ProcessConnectionIndicator(void)
{
	uint32_t now = HAL_GetTick();

	if ((statusEmerStop != 0U) || BoardLimitSwitch_IsPressed(emergency_stop_switch))
	{
		if ((now - panelLedTick) >= PACKET_COMM_LED_EMERGENCY_BLINK_MS)
		{
			panelLedTick = now;
			panelLedBlinkOn = !panelLedBlinkOn;
		}
		PacketComm_SetPanelLeds(panelLedBlinkOn, panelLedBlinkOn, panelLedBlinkOn);
		return;
	}

	if (cmdUartConnected)
	{
		connectionYellowOn = false;
		panelLedIndex = 0U;
		panelLedBlinkOn = false;
		PacketComm_SetPanelLeds(true, false, false);
		BoardRgb_Set(rgb_red, false);
		BoardRgb_Set(rgb_yellow, false);
		BoardRgb_Set(rgb_green, true);
		return;
	}

	if ((now - panelLedTick) >= PACKET_COMM_LED_RUN_MS)
	{
		panelLedTick = now;
		panelLedIndex = (uint8_t)((panelLedIndex + 1U) % 3U);
	}
	PacketComm_SetPanelLeds(panelLedIndex == 0U, panelLedIndex == 1U, panelLedIndex == 2U);

	if ((now - connectionIndicatorTick) >= PACKET_COMM_YELLOW_SLOW_BLINK_MS)
	{
		connectionIndicatorTick = now;
		connectionYellowOn = !connectionYellowOn;
	}

	BoardRgb_Set(rgb_red, false);
	BoardRgb_Set(rgb_green, false);
	BoardRgb_Set(rgb_yellow, connectionYellowOn);
}

HAL_StatusTypeDef PacketComm_SendStatus(void)
{
	static char log[1900];
	int length;
	uint16_t heMfcTargetCenti = PacketComm_FlowToCentiLpm(MFC_GetTargetFlow(He_MFC));
	uint16_t heMfcFeedbackCenti = PacketComm_FlowToCentiLpm(MFC_GetFeedbackFlow(He_MFC));
	plasma_feedback_measurement_t plasmaFeedback = {0};
	GasPressure_Data_t hePressure = {0};
	GasPressure_Data_t arPressure = {0};
	bool plasmaFeedbackReady;
	int32_t plasmaVppCenti;
	int32_t plasmaPeakCenti;
	int32_t plasmaRmsCenti;
	int32_t plasmaOffsetCenti;
	int32_t plasmaFreqDeci;
	int32_t plasmaHvVppCenti;
	int32_t plasmaHvPeakCenti;
	int32_t plasmaHvRmsCenti;
	PacketComm_RxContext *ctx = PacketComm_GetContext(PACKET_COMM_PORT_LOG);

	if ((ctx == NULL) || (ctx->uart == NULL))
	{
		return HAL_ERROR;
	}

	if (MFC_IsChannelEnabled(Ar_MFC))
	{
		(void)MFC_UpdateFeedback(Ar_MFC);
	}
	(void)MFC_UpdateFeedback(He_MFC);
	heMfcTargetCenti = PacketComm_FlowToCentiLpm(MFC_GetTargetFlow(He_MFC));
	heMfcFeedbackCenti = PacketComm_FlowToCentiLpm(MFC_GetFeedbackFlow(He_MFC));
	plasmaFeedbackReady = PlasmaFeedback_GetMeasurement(&plasmaFeedback);
	(void)GasPressure_GetData(GAS_HELIUM, &hePressure);
	(void)GasPressure_GetData(GAS_ARGON, &arPressure);
	plasmaVppCenti = PacketComm_FloatToCenti(plasmaFeedback.feedback_vpp_v);
	plasmaPeakCenti = PacketComm_FloatToCenti(plasmaFeedback.feedback_peak_v);
	plasmaRmsCenti = PacketComm_FloatToCenti(plasmaFeedback.feedback_rms_v);
	plasmaOffsetCenti = PacketComm_FloatToCenti(plasmaFeedback.feedback_offset_v);
	plasmaFreqDeci = PacketComm_FloatToDeci(plasmaFeedback.feedback_frequency_hz);
	plasmaHvVppCenti = PacketComm_FloatToCenti(plasmaFeedback.hv_vpp_kv);
	plasmaHvPeakCenti = PacketComm_FloatToCenti(plasmaFeedback.hv_peak_kv);
	plasmaHvRmsCenti = PacketComm_FloatToCenti(plasmaFeedback.hv_rms_kv);

	PacketComm_UpdateStatePacket();

	length = snprintf(log, sizeof(log),
					  "================ Plasma Controller Status ================\r\n"
					  "===System===\r\n"
					  "  Emergency Stop : %s\r\n"
					  "  Initial Limit  : %s  Max Limit      : %s  E-Stop Input: %s\r\n"
					  "===Power===\r\n"
					  "  Plasma    : %s  Voltage   : %s\r\n"
					  "===Gas Path===\r\n"
					  "  He_MFC : %s  Ar_MFC : %s\r\n"
					  "  He Valve: %s  Ar Valve: %s\r\n"
					  "  He Target  : %u.%02u L/min  He DAC Out : %u \r\n"
					  "  He Feedback: %u.%02u L/min  He ADC Input : %u\r\n"
					  "  He Tracking: %s\r\n"
					  "===Gas Pressure===\r\n"
					  "  He Pressure: %s  Online: %s  Valid: %s  Value=%lu.%02lu  Remain=%lu.%01lu%%\r\n"
					  "  Ar Pressure: %s  Online: %s  Valid: %s  Value=%lu.%02lu  Remain=%lu.%01lu%%\r\n"
					  "===Plasma Feedback===\r\n"
					  "  Ready      : %s  Signal: %s  Clip: %s  Freq: %s\r\n"
					  "  ADC Raw    : min=%u max=%u\r\n"
					  "  Feedback   : Vpp=%lu.%02lu V  Peak=%lu.%02lu V  RMS=%lu.%02lu V  Offset=%c%lu.%02lu V\r\n"
					  "  Frequency  : %lu.%01lu Hz\r\n"
					  "  HV Output  : Vpp=%lu.%02lu kV  Peak=%lu.%02lu kV  RMS=%lu.%02lu kV\r\n"
					  "===Auxiliary===\r\n"
					  "  Ctrl Aux Fan : %s, %u%%\r\n"
					  "  Device Fan   : %s, %u%%\r\n"
					  "  Ctrl Main Fan: %s, %u%%\r\n"
					  "  Buzzer 1     : %s  Buzzer 2    : %s\r\n"
					  "  RGB LED      : R=%s G=%s Y=%s\r\n"
					  "==========================================================\r\n",
					  PacketComm_OnOff(PacketComm_SendPacket.EmerStop != 0U),
					  PacketComm_PressedReleased(BoardLimitSwitch_IsPressed(initial_limit_switch)),
					  PacketComm_PressedReleased(BoardLimitSwitch_IsPressed(max_limit_switch)),
					  PacketComm_PressedReleased(BoardLimitSwitch_IsPressed(emergency_stop_switch)),
					  PacketComm_OnOff(BoardRelay_Get(plasma_relay)),
					  PacketComm_OnOff(BoardRelay_Get(vol_mod_relay)),
					  PacketComm_MfcPowerText(He_MFC),
					  PacketComm_MfcPowerText(Ar_MFC),
					  PacketComm_OnOff(BoardGasValve_Get(he_valve)),
					  PacketComm_OnOff(BoardGasValve_Get(ar_valve)),
					  heMfcTargetCenti / 100U,
					  heMfcTargetCenti % 100U,
					  MFC_GetDacCode(He_MFC),
					  heMfcFeedbackCenti / 100U,
					  heMfcFeedbackCenti % 100U,
					  MFC_GetAdcRaw(He_MFC),
					  PacketComm_MfcTrackingText(He_MFC),
					  PacketComm_ConfigText(hePressure.configured),
					  PacketComm_OnOff(hePressure.online),
					  PacketComm_ValidFault(hePressure.valid),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToCenti(hePressure.pressure)) / 100U),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToCenti(hePressure.pressure)) % 100U),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToDeci(hePressure.remaining_percent)) / 10U),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToDeci(hePressure.remaining_percent)) % 10U),
					  PacketComm_ConfigText(arPressure.configured),
					  PacketComm_OnOff(arPressure.online),
					  PacketComm_ValidFault(arPressure.valid),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToCenti(arPressure.pressure)) / 100U),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToCenti(arPressure.pressure)) % 100U),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToDeci(arPressure.remaining_percent)) / 10U),
					  (unsigned long)(PacketComm_Abs32(PacketComm_FloatToDeci(arPressure.remaining_percent)) % 10U),
					  PacketComm_OnOff(plasmaFeedbackReady),
					  PacketComm_ValidFault(plasmaFeedback.signal_valid),
					  PacketComm_OnOff(plasmaFeedback.clipped),
					  PacketComm_ValidFault(plasmaFeedback.frequency_valid),
					  plasmaFeedback.adc_min_raw,
					  plasmaFeedback.adc_max_raw,
					  (unsigned long)(PacketComm_Abs32(plasmaVppCenti) / 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaVppCenti) % 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaPeakCenti) / 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaPeakCenti) % 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaRmsCenti) / 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaRmsCenti) % 100U),
					  (plasmaOffsetCenti < 0) ? '-' : '+',
					  (unsigned long)(PacketComm_Abs32(plasmaOffsetCenti) / 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaOffsetCenti) % 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaFreqDeci) / 10U),
					  (unsigned long)(PacketComm_Abs32(plasmaFreqDeci) % 10U),
					  (unsigned long)(PacketComm_Abs32(plasmaHvVppCenti) / 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaHvVppCenti) % 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaHvPeakCenti) / 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaHvPeakCenti) % 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaHvRmsCenti) / 100U),
					  (unsigned long)(PacketComm_Abs32(plasmaHvRmsCenti) % 100U),
					  PacketComm_OnOff(BoardFan_Get(ctr_ass_fan)),
					  PacketComm_DutyToPercent(BoardFan_GetDuty(ctr_ass_fan)),
					  PacketComm_OnOff(BoardFan_Get(device_fan)),
					  PacketComm_DutyToPercent(BoardFan_GetDuty(device_fan)),
					  PacketComm_OnOff(BoardFan_Get(ctr_main_fan)),
					  PacketComm_DutyToPercent(BoardFan_GetDuty(ctr_main_fan)),
					  PacketComm_OnOff(BoardBuzzer_Get(buzzer_1)),
					  PacketComm_OnOff(BoardBuzzer_Get(buzzer_2)),
					  PacketComm_OnOff(BoardRgb_Get(rgb_red)),
					  PacketComm_OnOff(BoardRgb_Get(rgb_green)),
					  PacketComm_OnOff(BoardRgb_Get(rgb_yellow)));

	if (length <= 0)
	{
		return HAL_ERROR;
	}

	if ((uint32_t)length >= sizeof(log))
	{
		length = sizeof(log) - 1;
	}

	return HAL_UART_Transmit(ctx->uart,
							 (uint8_t *)log,
							 (uint16_t)length,
							 PacketComm_TxTimeoutMs((uint16_t)length));
}

HAL_StatusTypeDef PacketComm_SendStatePacket(PacketComm_Port port)
{
	uint8_t packet[PACKET_COMM_STATE_SIZE] = {0};
	PacketComm_RxContext *ctx = PacketComm_GetContext(port);

	if ((ctx == NULL) || (ctx->uart == NULL))
	{
		return HAL_ERROR;
	}

	PacketComm_UpdateStatePacket();
	PacketComm_PackState(packet);
	WriteFloatLE(&packet[16], PacketComm_GetPlasmaFeedbackVpp());
	WriteFloatLE(&packet[20], PacketComm_GetGasPressureValue(GAS_HELIUM));
	WriteFloatLE(&packet[24], PacketComm_GetGasPressureValue(GAS_ARGON));

	return HAL_UART_Transmit(ctx->uart,
							 packet,
							 PACKET_COMM_STATE_SIZE,
							 PacketComm_TxTimeoutMs(PACKET_COMM_STATE_SIZE));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	PacketComm_RxContext *ctx = PacketComm_FindContext(huart);

	if (ctx != NULL)
	{
		if (ctx->rxLen < PACKET_COMM_RX_BUF_SIZE)
		{
			ctx->rxBuf[ctx->rxLen++] = ctx->rxByte;
		}
		else
		{
			ctx->rxLen = 0;
			ctx->rxBuf[ctx->rxLen++] = ctx->rxByte;
		}

		HAL_UART_Receive_IT(ctx->uart, &ctx->rxByte, 1);
	}
	else
	{
		(void)Modbus_HandleUartRxCpltCallback(huart);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	PacketComm_RxContext *ctx = PacketComm_FindContext(huart);

	if (ctx != NULL)
	{
		HAL_UART_Receive_IT(ctx->uart, &ctx->rxByte, 1);
	}
	else
	{
		(void)Modbus_HandleUartErrorCallback(huart);
	}
}
