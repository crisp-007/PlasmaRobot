#include "cmd.h"

// 初始化
uint8_t recv_buf[RX_BUFFER_SIZE] = {0};  // USART1命令串口缓冲区
uint8_t log_buf[RX_BUFFER_SIZE] = {0};   // USART2日志串口缓冲区
uint8_t cmd_buf[RX_BUFFER_SIZE] = {0};
bool PackReady = false;
uint16_t rxIndex = 0;      // USART1索引
uint16_t logIndex = 0;     // USART2索引

bool ifHead = false;
bool ifComplete = false;

DataPacket SendPacket = {0}; // 要发送的数据包
DataPacket RecvPacket = {0}; // 要接收的数据包

// RecvPacket.header=cmd_buf[0]<<8|cmd_buf[1];
// RecvPacket.footer=cmd_buf[9];

// 处理串口接收的数据，拿到完整的包后，取出包中的数据
void DealPortData(bool iflog)
{
	if (iflog)
	{
		printf("当前索引值：%d\n", rxIndex);
		printf("当前接收缓冲区: ");
		for (int i = 0; i < rxIndex; i++)
		{
			printf("%02X ", recv_buf[i]);
		}
		printf("\n");
		
		if (ifComplete)
		{
			printf("当前命令缓冲区: ");
			for (int i = 0; i < 16; i++)
			{
				printf("%02X ", cmd_buf[i]);
			}
		}
		printf("\n");
		delay_ms(2000);
	}

	if (rxIndex >= 2)
	{
		if ((recv_buf[1] << 8 | recv_buf[0]) == 0xFEFF)
		{
			ifHead = true;

			if (rxIndex >= 16)
			{
				u32 addCheck = 0;
				// 包尾检测
				for (int i = 0; i <= 11; i++)
				{
					addCheck += recv_buf[i];
				}
				uint32_t real_add = (recv_buf[15] << 24 | recv_buf[14] << 16 | recv_buf[13] << 8 | recv_buf[12]);
				if (addCheck == real_add)
				{
					// 通过和校验，包正确
					ifComplete = true;
					printf("通过和校验：%02X|%02x\n", addCheck, real_add);
					memmove(&cmd_buf[0], &recv_buf[0], 16);
					memmove(&recv_buf[0], &recv_buf[16], sizeof(recv_buf) - 16);
					rxIndex -= 16;

					// 开始处理数据
					// 将数据包中的数据分发到各个对应变量
					RecvPacket.EmerStop = cmd_buf[2];
					RecvPacket.VolRelay = cmd_buf[3];
					RecvPacket.VolOutValue = cmd_buf[5] << 8 | cmd_buf[4];
					RecvPacket.HeFLOWRelay = cmd_buf[6];
					RecvPacket.HeOutValue = cmd_buf[8] << 8 | cmd_buf[7];
					RecvPacket.ArFLOWRelay = cmd_buf[9];
					RecvPacket.ArOutValue = cmd_buf[11] << 8 | cmd_buf[10];

					if (iflog)
						DeviceStateLog(); // 打印收到的各个变量

					PackReady = true; // 可以进行处理

					ifComplete = false; // 包处理完成，标志复位
				}
				else
				{
					// 未通过和校验，删除当前这个包
					ifComplete = false;
					// PackReady = true;
					printf("未通过和校验：%02X|%02x\n", addCheck, real_add);
					memmove(&recv_buf[0], &recv_buf[16], sizeof(recv_buf) - 16);
					rxIndex -= 16;
				}
			}
		}
		else
		{
			// 去掉头一位
			//  recv_buf[0]=recv_buf[1];
			printf("未找到包头！\r\n");
			while (rxIndex >= 2)
			{
				ifHead = false;
				rxIndex -= 1;
				memmove(&recv_buf[0], &recv_buf[0] + 1, sizeof(recv_buf) - 1);
				if ((recv_buf[1] << 8 | recv_buf[0]) == 0xFEFF)
					break;
			}
			ifHead = true;
		}
	}
}

u8 CmdDeal(bool ready)
{
	// 检测包解析是否完成
	if (!ready)
		return 0;

	printf("========================DataPacket Deal=======================\n");
	// 检测上位机是否发布急停信号
	if (RecvPacket.EmerStop == 0)
	{
		// 没有检测到急停信号

		
		
	}
	else if (RecvPacket.EmerStop > 0)
	{
		
	}


	// TODO:处理各个逻辑
	printf("Param Setting！\n");
	// 1、等离子电源继电器控制
	uint8_t plasmaRelay = (RecvPacket.VolRelay >> 4) & 0x0F; // 高4位
	uint8_t volModRelay = RecvPacket.VolRelay & 0x0F;        // 低4位
	
	if (plasmaRelay > 0)
	{
		OpenPlasmaVol();
		printf("PlasmaRelay-ON\n");
	}
	else
	{
		ClosePlasmaVol();
		printf("PlasmaRelay-OFF\n");
	}

	// 2、调压器继电器控制
	if (volModRelay > 0)
	{
		OpenModVol();
		printf("VolModRelay-ON\n");
	}
	else
	{
		CloseModVol();
		printf("VolModRelay-OFF\n");
	}

	// 3、氦气模块管理
	uint8_t heFlowMeter = (RecvPacket.HeFLOWRelay >> 4) & 0x0F; // 高4位控制流量计
	uint8_t heValve = RecvPacket.HeFLOWRelay & 0x0F;           // 低4位控制电磁阀
	
	if (heFlowMeter > 0 || heValve > 0)
	{
		// 氦气设备控制
		if (heFlowMeter > 0) {
			OpenHeFlowMeter();
			printf("HeFlowMeter-ON\n");
		} else {
			CloseHeFlowMeter();
			printf("HeFlowMeter-OFF\n");
		}
		
		if (heValve > 0) {
			OpenHeGasValve();
			printf("HeValve-ON\n");
		} else {
			CloseHeGasValve();
			printf("HeValve-OFF\n");
		}
		
		// 氦气流量设定
		float CurHeFlow = (float)RecvPacket.HeOutValue / 4096 * 3.3 / 5 * 30;
		printf("HeFlow:%d || %.2f L/min\n", RecvPacket.HeOutValue, CurHeFlow);
		Set_HeFlow_Value(RecvPacket.HeOutValue);
	}
	else
	{
		CloseHeGasDevice();
		Set_HeFlow_Value(0);
		printf("HeFlow forbid\n");
	}
	
	// 4、氩气模块管理
	uint8_t arFlowMeter = (RecvPacket.ArFLOWRelay >> 4) & 0x0F; // 高4位控制流量计
	uint8_t arValve = RecvPacket.ArFLOWRelay & 0x0F;           // 低4位控制电磁阀
	
	if (arFlowMeter > 0 || arValve > 0)
	{
		// 氩气设备控制
		if (arFlowMeter > 0) {
			OpenArFlowMeter();
			printf("ArFlowMeter-ON\n");
		} else {
			CloseArFlowMeter();
			printf("ArFlowMeter-OFF\n");
		}
		
		if (arValve > 0) {
			OpenArGasValve();
			printf("ArValve-ON\n");
		} else {
			CloseArGasValve();
			printf("ArValve-OFF\n");
		}
		
		// 氩气流量设定
		float CurArFlow = (float)RecvPacket.ArOutValue / 4096 * 3.3 / 5 * 10;
		printf("ArFlow:%d || %.2f L/min\n", RecvPacket.ArOutValue, CurArFlow);
		Set_ArFlow_Value(RecvPacket.ArOutValue);
	}
	else
	{
		CloseArGasDevice();
		Set_ArFlow_Value(0);
		printf("ArFlow forbid\n");
	}
	// 4、等离子电源模拟量检测

	// 5、调压器步进量-理论电压值

	PackReady = false;
	printf("========================DataPacket Down=====================\n");
	// 将包是否准备就绪标志复位
	
	return 1;
}

u8 SendPack(void)
{

	SendPacket.header = 0xFEFF;

	if (GetemStopState())
	{
		SendPacket.EmerStop = 0xf8;
	}
	else
	{
		SendPacket.EmerStop = RecvPacket.EmerStop;
	}

	// 等离子电源继电器状态（高4位：等离子电源，低4位：调压器）
	uint8_t plasmaState = (GetVolRelayState() == 1) ? 0x10 : 0x00;
	uint8_t volModState = (GetVolModRelayState() == 1) ? 0x01 : 0x00;
	SendPacket.VolRelay = plasmaState | volModState;
	
	// 等离子电源输出值
	SendPacket.VolOutValue = ADC_ConvertedValue[0]; // 等离子电源ADC值
	
	// 氦气流量继电器状态（高4位：流量计，低4位：电磁阀）
	uint8_t heFlowMeterState = (GetHeFlowMeterState() == 1) ? 0x10 : 0x00;
	uint8_t heValveState = (GetHeGasValveState() == 1) ? 0x01 : 0x00;
	SendPacket.HeFLOWRelay = heFlowMeterState | heValveState;
	
	// 氦气输出值
	SendPacket.HeOutValue = ADC_ConvertedValue[1]; // 氦气流量ADC值
	
	// 氩气流量继电器状态（高4位：流量计，低4位：电磁阀）
	uint8_t arFlowMeterState = (GetArFlowMeterState() == 1) ? 0x10 : 0x00;
	uint8_t arValveState = (GetArGasValveState() == 1) ? 0x01 : 0x00;
	SendPacket.ArFLOWRelay = arFlowMeterState | arValveState;
	
	// 氩气输出值
	SendPacket.ArOutValue = ADC_ConvertedValue[2]; // 氩气流量ADC值
	
	// footer校验和
	SendPacket.footer = SendPacket.header + SendPacket.EmerStop +
						SendPacket.VolRelay + SendPacket.VolOutValue +
						SendPacket.HeFLOWRelay + SendPacket.HeOutValue +
						SendPacket.ArFLOWRelay + SendPacket.ArOutValue;

	// 1、发送包头
	USART_SendU16(SendPacket.header); // SendPacket.header 0XFEFF
	// 2、发送数据
	USART_SendU8(SendPacket.EmerStop);

	// 等离子电源相关
	USART_SendU8(SendPacket.VolRelay);
	USART_SendU16(SendPacket.VolOutValue);

	// 氦气相关
	USART_SendU8(SendPacket.HeFLOWRelay);
	USART_SendU16(SendPacket.HeOutValue);
	
	// 氩气相关
	USART_SendU8(SendPacket.ArFLOWRelay);
	USART_SendU16(SendPacket.ArOutValue);
	
	// 3、发送包尾
	USART_SendU32(SendPacket.footer);
	return 1;
}

void DeviceStateLog(void)
{
	printf("========================DataPacket Show=====================\n");
	printf("Emerg-stop:%d\n", RecvPacket.EmerStop);

	// 等离子电源相关
	uint8_t plasmaRelay = (RecvPacket.VolRelay >> 4) & 0x0F;
	uint8_t volModRelay = RecvPacket.VolRelay & 0x0F;
	printf("PlasmaRelay:%d, VolModRelay:%d\n", plasmaRelay, volModRelay);
	printf("VolOutValue:%d\n", RecvPacket.VolOutValue);

	// 氦气相关
	uint8_t heFlowMeter = (RecvPacket.HeFLOWRelay >> 4) & 0x0F;
	uint8_t heValve = RecvPacket.HeFLOWRelay & 0x0F;
	printf("HeFlowMeter:%d, HeValve:%d\n", heFlowMeter, heValve);
	printf("HeFlow:%d||%.2f L/min\n", RecvPacket.HeOutValue, (float)RecvPacket.HeOutValue / 4096 * 3.3 / 5 * 10);

	// 氩气相关
	uint8_t arFlowMeter = (RecvPacket.ArFLOWRelay >> 4) & 0x0F;
	uint8_t arValve = RecvPacket.ArFLOWRelay & 0x0F;
	printf("ArFlowMeter:%d, ArValve:%d\n", arFlowMeter, arValve);
	printf("ArFlow:%d||%.2f L/min\n", RecvPacket.ArOutValue, (float)RecvPacket.ArOutValue / 4096 * 3.3 / 5 * 10);
	printf("========================DataPacket Show=====================\n");
}
