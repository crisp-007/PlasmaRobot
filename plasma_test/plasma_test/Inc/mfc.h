#ifndef __MFC_H__
#define __MFC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  /* Argon MFC: PC2/SWRAY1, PA4 DAC, PB0 ADC. Current hardware issue, disabled by default. */
  Ar_MFC = 0,
  /* Helium MFC: PC3/SWRAY2, PA5 DAC, PB1 ADC. Current active MFC channel. */
  He_MFC,
  MFC_CHANNEL_COUNT
} mfc_channel_t;

typedef enum
{
  MFC_RESULT_OK = 0,
  /* 通道号越界，例如 >= MFC_CHANNEL_COUNT。 */
  MFC_RESULT_INVALID_CHANNEL,
  /* 通道被配置为禁用，例如当前 Ar_MFC。 */
  MFC_RESULT_CHANNEL_DISABLED,
  /* 配置不合法，例如满量程 <= 0、电压范围不对或 DAC/ADC 硬件参数不匹配。 */
  MFC_RESULT_INVALID_CONFIG,
  /* 通道未上电时尝试设置非零流量。 */
  MFC_RESULT_NOT_POWERED,
  /* 底层 ADC/DAC/GPIO 操作失败或超时。 */
  MFC_RESULT_HAL_ERROR
} mfc_result_t;

typedef enum
{
  MFC_POWER_OFF = 0,
  MFC_POWER_STARTING,
  MFC_POWER_ON,
  MFC_POWER_STOPPING,
  MFC_POWER_FAULT
} mfc_power_state_t;

mfc_result_t MFC_Init(void);

bool MFC_IsChannelEnabled(mfc_channel_t channel);

/* 上电会先把该通道 DAC 设为最小流量，再吸合 MFC 电源继电器。 */
mfc_result_t MFC_PowerOn(mfc_channel_t channel);
/* 断电会先把目标流量降为最小值，再释放 MFC 电源继电器。 */
mfc_result_t MFC_PowerOff(mfc_channel_t channel);
bool MFC_IsPowered(mfc_channel_t channel);
mfc_power_state_t MFC_GetPowerState(mfc_channel_t channel);

/* 修改 MFC 满量程。例如 Ar_MFC 为 10 L/min，He_MFC 为 30 L/min。 */
mfc_result_t MFC_SetFullScaleFlow(mfc_channel_t channel, float full_scale_flow_lpm);
float MFC_GetFullScaleFlow(mfc_channel_t channel);

/* 设置目标流量，单位 L/min。未上电时只允许设置 0。 */
mfc_result_t MFC_SetFlow(mfc_channel_t channel, float target_flow_lpm);
/* 读取 ADC 反馈并换算成 L/min；断电状态下不会进行有效反馈判断。 */
mfc_result_t MFC_UpdateFeedback(mfc_channel_t channel);

float MFC_GetTargetFlow(mfc_channel_t channel);
float MFC_GetInstantFeedbackFlow(mfc_channel_t channel);
float MFC_GetFeedbackFlow(mfc_channel_t channel);
float MFC_GetTargetVoltage(mfc_channel_t channel);
float MFC_GetFeedbackVoltage(mfc_channel_t channel);

uint16_t MFC_GetAdcRaw(mfc_channel_t channel);
uint16_t MFC_GetDacCode(mfc_channel_t channel);

bool MFC_IsTrackingNormal(mfc_channel_t channel);
bool MFC_IsFeedbackValid(mfc_channel_t channel);
bool MFC_HasFault(mfc_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __MFC_H__ */
