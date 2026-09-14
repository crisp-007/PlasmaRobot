#include "mfc.h"

#include <stddef.h>

#define AR_MFC_PWR_EN_GPIO_PORT  GPIOC
#define AR_MFC_PWR_EN_GPIO_PIN   GPIO_PIN_2
#define HE_MFC_PWR_EN_GPIO_PORT  GPIOC
#define HE_MFC_PWR_EN_GPIO_PIN   GPIO_PIN_3

/* ADC 单次轮询超时。若模拟输入异常或 ADC 没有完成转换，避免主循环永久卡住。 */
#define MFC_ADC_TIMEOUT_MS     5U
/* 反馈流量一阶低通滤波系数。0.20 表示新采样占 20%，历史值占 80%。 */
#define MFC_FILTER_ALPHA       0.60f

typedef enum
{
  MFC_CHANNEL_DISABLED = 0,
  MFC_CHANNEL_ENABLED
} mfc_channel_enable_t;

typedef struct
{
  /* MFC 规格参数。换 MFC 型号时最常改的是 flow_max_lpm。 */
  float flow_min_lpm;
  /* 满量程流量，单位 L/min。例如 10 L/min 的 MFC 填 10.0f。 */
  float flow_max_lpm;
  /* MFC 设定输入电压范围。常见模拟 MFC 为 0-5 V，也可能是 1-5 V。 */
  float set_voltage_min_v;
  float set_voltage_max_v;
  /* MFC 反馈输出电压范围。通常和设定一样为 0-5 V。 */
  float feedback_voltage_min_v;
  float feedback_voltage_max_v;
} mfc_device_spec_t;

typedef struct
{
  /* DAC 参考电压，通常等于 VDDA，默认按 3.3 V 计算。 */
  float vref_v;
  /* 12 位 DAC 最大码值为 4095。 */
  uint16_t resolution_max;
  /* 板卡对外的 MFC 设定电压范围，本板通过运放输出 0-5 V。 */
  float interface_output_min_v;
  float interface_output_max_v;
  /* DAC 外部放大倍数。本板 DAC 输出经 2 倍同相放大，因此填 2.0。 */
  float external_gain;
  /* 外部模拟链路偏置，当前硬件无偏置填 0。 */
  float external_offset_v;
  /* MCU DAC 引脚目标电压安全范围。本板 5 V 接口经 2 倍放大，所以满量程只需 2.5 V。 */
  float mcu_dac_min_v;
  float mcu_dac_max_v;
  /* 内部使用的 DAC 通道编号：1=PA4/DAC_OUT1，2=PA5/DAC_OUT2。 */
  uint32_t dac_channel;
} mfc_dac_hw_config_t;

typedef struct
{
  /* ADC 参考电压，通常等于 VDDA。 */
  float vref_v;
  /* 12 位 ADC 最大原始值为 4095。 */
  uint16_t resolution_max;
  /* 板卡接口侧反馈电压范围，即 MFC 输出的 0-5 V。 */
  float interface_input_min_v;
  float interface_input_max_v;
  /* ADC 前端分压比例。本板 10k/10k 二分压，MCU 引脚电压 = 接口电压 * 0.5。 */
  float divider_ratio;
  /* ADC 模拟链路偏置，当前硬件无偏置填 0。 */
  float analog_offset_v;
  /* MCU ADC 引脚目标电压范围。本板 5 V 反馈二分压后最高约 2.5 V。 */
  float mcu_adc_min_v;
  float mcu_adc_max_v;
  /* ADC 通道编号：8=PB0/ADC12_IN8，9=PB1/ADC12_IN9。 */
  uint32_t adc_channel;
} mfc_adc_hw_config_t;

typedef struct
{
  /* MFC 电源继电器控制脚。Ar_MFC=PC2/SWRAY1，He_MFC=PC3/SWRAY2。 */
  GPIO_TypeDef *gpio_port;
  uint16_t gpio_pin;
  /* 继电器吸合有效电平。当前原理图按高电平吸合配置为 GPIO_PIN_SET。 */
  GPIO_PinState power_active_level;
  /* 上电后等待 MFC 稳定的时间。太短可能导致刚上电就设置流量不稳定。 */
  uint32_t power_on_delay_ms;
  /* 断电前给零流量命令后的等待时间。 */
  uint32_t power_off_delay_ms;
} mfc_power_hw_config_t;

typedef struct
{
  /* DAC 标定：最终 DAC 码值 = 理论码值 * dac_gain + dac_offset_code。 */
  float dac_gain;
  float dac_offset_code;
  /* ADC 标定：接口反馈电压 = 理论反馈电压 * adc_gain + adc_offset_v。 */
  float adc_gain;
  float adc_offset_v;
} mfc_calibration_t;

typedef struct
{
  /* 跟踪判断允许的固定误差，单位 L/min。 */
  float tracking_abs_tolerance_lpm;
  /* 跟踪判断允许的相对误差。例如 0.05 表示满量程的 5%。 */
  float tracking_relative_tolerance;
  /* 目标接近 0 时不做严格跟踪判断，避免零点噪声误报。 */
  float zero_deadband_lpm;
  /* 预留：目标变化后允许 MFC 建立流量的时间。 */
  uint32_t response_timeout_ms;
} mfc_diagnostic_config_t;

typedef struct
{
  mfc_channel_enable_t channel_enable;
  mfc_device_spec_t device;
  mfc_dac_hw_config_t dac_hw;
  mfc_adc_hw_config_t adc_hw;
  mfc_power_hw_config_t power_hw;
  mfc_calibration_t calibration;
  mfc_diagnostic_config_t diagnostics;
} mfc_channel_config_t;

typedef struct
{
  float target_flow_lpm;
  float feedback_flow_lpm;
  float filtered_feedback_flow_lpm;
  float target_voltage_v;
  float feedback_voltage_v;
  uint16_t dac_code;
  uint16_t adc_raw;
  mfc_power_state_t power_state;
  bool tracking_ok;
  bool feedback_valid;
  bool feedback_filter_ready;
  bool fault;
} mfc_status_t;

typedef struct
{
  const char *name;
  mfc_channel_config_t config;
  mfc_status_t status;
} mfc_device_t;

static mfc_device_t g_mfc[MFC_CHANNEL_COUNT] =
{
  [Ar_MFC] =
  {
    .name = "Ar_MFC",
    .config =
    {
      /* Ar_MFC 当前 PC2/SWRAY1 接线存在问题，按文档要求暂时禁用。修复后改成 MFC_CHANNEL_ENABLED。 */
      .channel_enable = MFC_CHANNEL_DISABLED,
      .device =
      {
        .flow_min_lpm = 0.0f,
        /* Ar_MFC 当前型号满量程为 10 L/min；通道暂时禁用，但配置先按实物保留。 */
        .flow_max_lpm = 10.0f,
        .set_voltage_min_v = 0.0f,
        .set_voltage_max_v = 5.0f,
        .feedback_voltage_min_v = 0.0f,
        .feedback_voltage_max_v = 5.0f
      },
      .dac_hw =
      {
        .vref_v = 3.3f,
        .resolution_max = 4095U,
        .interface_output_min_v = 0.0f,
        .interface_output_max_v = 5.0f,
        .external_gain = 2.0f,
        .external_offset_v = 0.0f,
        .mcu_dac_min_v = 0.0f,
        /* 5 V 接口经 2 倍运放放大得到，所以 MCU DAC 满量程目标为 2.5 V，约 3102 码。 */
        .mcu_dac_max_v = 2.5f,
        .dac_channel = 1U
      },
      .adc_hw =
      {
        .vref_v = 3.3f,
        .resolution_max = 4095U,
        .interface_input_min_v = 0.0f,
        .interface_input_max_v = 5.0f,
        .divider_ratio = 0.5f,
        .analog_offset_v = 0.0f,
        .mcu_adc_min_v = 0.0f,
        /* 5 V 反馈经二分压后进入 ADC，MCU 侧最高约 2.5 V。 */
        .mcu_adc_max_v = 2.5f,
        .adc_channel = 8U
      },
      .power_hw =
      {
        .gpio_port = AR_MFC_PWR_EN_GPIO_PORT,
        .gpio_pin = AR_MFC_PWR_EN_GPIO_PIN,
        .power_active_level = GPIO_PIN_SET,
        .power_on_delay_ms = 1000U,
        .power_off_delay_ms = 100U
      },
      .calibration =
      {
        .dac_gain = 1.0f,
        .dac_offset_code = 0.0f,
        .adc_gain = 1.0f,
        .adc_offset_v = 0.0f
      },
      .diagnostics =
      {
        .tracking_abs_tolerance_lpm = 0.5f,
        .tracking_relative_tolerance = 0.05f,
        .zero_deadband_lpm = 0.1f,
        .response_timeout_ms = 2000U
      }
    }
  },
  [He_MFC] =
  {
    .name = "He_MFC",
    .config =
    {
      /* He_MFC 当前有效。PC3/SWRAY2 控制上电，PA5 输出设定，PB1 读取反馈。 */
      .channel_enable = MFC_CHANNEL_ENABLED,
      .device =
      {
        .flow_min_lpm = 0.0f,
        /* He_MFC 当前按 30 L/min 满量程配置。若实际型号不同，优先修改这里。 */
        .flow_max_lpm = 30.0f,
        .set_voltage_min_v = 0.0f,
        .set_voltage_max_v = 5.0f,
        .feedback_voltage_min_v = 0.0f,
        .feedback_voltage_max_v = 5.0f
      },
      .dac_hw =
      {
        .vref_v = 3.3f,
        .resolution_max = 4095U,
        .interface_output_min_v = 0.0f,
        .interface_output_max_v = 5.0f,
        .external_gain = 2.0f,
        .external_offset_v = 0.0f,
        .mcu_dac_min_v = 0.0f,
        .mcu_dac_max_v = 2.5f,
        .dac_channel = 2U
      },
      .adc_hw =
      {
        .vref_v = 3.3f,
        .resolution_max = 4095U,
        .interface_input_min_v = 0.0f,
        .interface_input_max_v = 5.0f,
        .divider_ratio = 0.5f,
        .analog_offset_v = 0.0f,
        .mcu_adc_min_v = 0.0f,
        .mcu_adc_max_v = 2.5f,
        .adc_channel = 9U
      },
      .power_hw =
      {
        .gpio_port = HE_MFC_PWR_EN_GPIO_PORT,
        .gpio_pin = HE_MFC_PWR_EN_GPIO_PIN,
        .power_active_level = GPIO_PIN_SET,
        .power_on_delay_ms = 1000U,
        .power_off_delay_ms = 100U
      },
      .calibration =
      {
        .dac_gain = 1.0f,
        .dac_offset_code = 0.0f,
        .adc_gain = 1.0f,
        .adc_offset_v = 0.0f
      },
      .diagnostics =
      {
        .tracking_abs_tolerance_lpm = 0.3f,
        .tracking_relative_tolerance = 0.05f,
        .zero_deadband_lpm = 0.1f,
        .response_timeout_ms = 2000U
      }
    }
  }
};

static float MFC_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static GPIO_PinState MFC_GetPowerOffLevel(const mfc_power_hw_config_t *power)
{
  return (power->power_active_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static mfc_result_t MFC_CheckChannel(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return MFC_RESULT_INVALID_CHANNEL;
  }

  if (g_mfc[channel].config.channel_enable != MFC_CHANNEL_ENABLED)
  {
    return MFC_RESULT_CHANNEL_DISABLED;
  }

  return MFC_RESULT_OK;
}

static bool MFC_ValidateConfig(const mfc_channel_config_t *cfg)
{
  float required_mcu_dac_v;
  float max_adc_pin_v;

  if (cfg == NULL)
  {
    return false;
  }

  if (cfg->device.flow_max_lpm <= cfg->device.flow_min_lpm)
  {
    return false;
  }

  if (cfg->device.set_voltage_max_v <= cfg->device.set_voltage_min_v)
  {
    return false;
  }

  if (cfg->device.feedback_voltage_max_v <= cfg->device.feedback_voltage_min_v)
  {
    return false;
  }

  if ((cfg->dac_hw.external_gain <= 0.0f) ||
      (cfg->dac_hw.vref_v <= 0.0f) ||
      (cfg->dac_hw.resolution_max == 0U))
  {
    return false;
  }

  if ((cfg->adc_hw.divider_ratio <= 0.0f) ||
      (cfg->adc_hw.divider_ratio > 1.0f) ||
      (cfg->adc_hw.vref_v <= 0.0f) ||
      (cfg->adc_hw.resolution_max == 0U))
  {
    return false;
  }

  required_mcu_dac_v =
      (cfg->device.set_voltage_max_v - cfg->dac_hw.external_offset_v) /
      cfg->dac_hw.external_gain;

  if ((required_mcu_dac_v < cfg->dac_hw.mcu_dac_min_v) ||
      (required_mcu_dac_v > cfg->dac_hw.mcu_dac_max_v))
  {
    return false;
  }

  max_adc_pin_v =
      cfg->device.feedback_voltage_max_v * cfg->adc_hw.divider_ratio +
      cfg->adc_hw.analog_offset_v;

  if (max_adc_pin_v > cfg->adc_hw.mcu_adc_max_v)
  {
    return false;
  }

  return true;
}

static void MFC_AnalogHwInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_DAC_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(AR_MFC_PWR_EN_GPIO_PORT, AR_MFC_PWR_EN_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HE_MFC_PWR_EN_GPIO_PORT, HE_MFC_PWR_EN_GPIO_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = AR_MFC_PWR_EN_GPIO_PIN | HE_MFC_PWR_EN_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  DAC->CR = DAC_CR_EN1 | DAC_CR_EN2;
  DAC->DHR12R1 = 0U;
  DAC->DHR12R2 = 0U;

  ADC2->CR1 = 0U;
  ADC->CCR &= ~ADC_CCR_ADCPRE;
  ADC->CCR |= ADC_CCR_ADCPRE_0;
  ADC2->CR2 = ADC_CR2_ADON;
  ADC2->SQR1 = 0U;
  ADC2->SMPR2 |= ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;
}

static void MFC_WriteDacCode(uint32_t dac_channel, uint16_t dac_code)
{
  if (dac_code > 4095U)
  {
    dac_code = 4095U;
  }

  if (dac_channel == 1U)
  {
    DAC->DHR12R1 = dac_code;
  }
  else
  {
    DAC->DHR12R2 = dac_code;
  }
}

static bool MFC_ReadAdcRaw(uint32_t adc_channel, uint16_t *adc_raw)
{
  uint32_t start_tick;

  if ((adc_raw == NULL) || (adc_channel > 18U))
  {
    return false;
  }

  ADC2->SQR3 = adc_channel;
  ADC2->SR = 0U;
  ADC2->CR2 |= ADC_CR2_SWSTART;

  start_tick = HAL_GetTick();
  while ((ADC2->SR & ADC_SR_EOC) == 0U)
  {
    if ((HAL_GetTick() - start_tick) > MFC_ADC_TIMEOUT_MS)
    {
      return false;
    }
  }

  *adc_raw = (uint16_t)(ADC2->DR & 0x0FFFU);
  return true;
}

static uint16_t MFC_FlowToDacCode(const mfc_channel_config_t *cfg, float target_flow_lpm)
{
  float flow_ratio;
  float interface_voltage_v;
  float mcu_dac_voltage_v;
  float dac_code;

  if (!MFC_ValidateConfig(cfg))
  {
    return 0U;
  }

  target_flow_lpm = MFC_ClampFloat(target_flow_lpm,
                                   cfg->device.flow_min_lpm,
                                   cfg->device.flow_max_lpm);

  flow_ratio =
      (target_flow_lpm - cfg->device.flow_min_lpm) /
      (cfg->device.flow_max_lpm - cfg->device.flow_min_lpm);

  interface_voltage_v =
      cfg->device.set_voltage_min_v +
      flow_ratio * (cfg->device.set_voltage_max_v - cfg->device.set_voltage_min_v);

  interface_voltage_v = MFC_ClampFloat(interface_voltage_v,
                                       cfg->dac_hw.interface_output_min_v,
                                       cfg->dac_hw.interface_output_max_v);

  mcu_dac_voltage_v =
      (interface_voltage_v - cfg->dac_hw.external_offset_v) /
      cfg->dac_hw.external_gain;

  mcu_dac_voltage_v = MFC_ClampFloat(mcu_dac_voltage_v,
                                     cfg->dac_hw.mcu_dac_min_v,
                                     cfg->dac_hw.mcu_dac_max_v);

  dac_code =
      mcu_dac_voltage_v /
      cfg->dac_hw.vref_v *
      (float)cfg->dac_hw.resolution_max;

  dac_code =
      dac_code * cfg->calibration.dac_gain +
      cfg->calibration.dac_offset_code;

  dac_code = MFC_ClampFloat(dac_code, 0.0f, (float)cfg->dac_hw.resolution_max);

  return (uint16_t)(dac_code + 0.5f);
}

static float MFC_DacCodeToInterfaceVoltage(const mfc_channel_config_t *cfg, uint16_t dac_code)
{
  float mcu_dac_voltage_v;

  if ((cfg == NULL) || (cfg->dac_hw.resolution_max == 0U) || (cfg->dac_hw.vref_v <= 0.0f))
  {
    return 0.0f;
  }

  mcu_dac_voltage_v =
      (float)dac_code /
      (float)cfg->dac_hw.resolution_max *
      cfg->dac_hw.vref_v;

  return mcu_dac_voltage_v * cfg->dac_hw.external_gain + cfg->dac_hw.external_offset_v;
}

static float MFC_AdcCodeToFlow(const mfc_channel_config_t *cfg,
                               uint16_t adc_raw,
                               float *feedback_voltage_v)
{
  float adc_pin_voltage_v;
  float interface_voltage_v;
  float voltage_ratio;
  float flow_lpm;

  if (!MFC_ValidateConfig(cfg))
  {
    if (feedback_voltage_v != NULL)
    {
      *feedback_voltage_v = 0.0f;
    }
    return 0.0f;
  }

  if (adc_raw > cfg->adc_hw.resolution_max)
  {
    adc_raw = cfg->adc_hw.resolution_max;
  }

  adc_pin_voltage_v =
      (float)adc_raw /
      (float)cfg->adc_hw.resolution_max *
      cfg->adc_hw.vref_v;

  interface_voltage_v =
      (adc_pin_voltage_v - cfg->adc_hw.analog_offset_v) /
      cfg->adc_hw.divider_ratio;

  interface_voltage_v =
      interface_voltage_v * cfg->calibration.adc_gain +
      cfg->calibration.adc_offset_v;

  interface_voltage_v = MFC_ClampFloat(interface_voltage_v,
                                       cfg->adc_hw.interface_input_min_v,
                                       cfg->adc_hw.interface_input_max_v);

  voltage_ratio =
      (interface_voltage_v - cfg->device.feedback_voltage_min_v) /
      (cfg->device.feedback_voltage_max_v - cfg->device.feedback_voltage_min_v);

  voltage_ratio = MFC_ClampFloat(voltage_ratio, 0.0f, 1.0f);
  flow_lpm =
      cfg->device.flow_min_lpm +
      voltage_ratio * (cfg->device.flow_max_lpm - cfg->device.flow_min_lpm);

  if (feedback_voltage_v != NULL)
  {
    *feedback_voltage_v = interface_voltage_v;
  }

  return flow_lpm;
}

static bool MFC_CheckTracking(const mfc_device_t *device)
{
  float abs_tol;
  float rel_tol;
  float allowed_error;
  float error;

  if ((device == NULL) || !device->status.feedback_valid)
  {
    return false;
  }

  if (device->status.power_state != MFC_POWER_ON)
  {
    return true;
  }

  if (device->status.target_flow_lpm <= device->config.diagnostics.zero_deadband_lpm)
  {
    return true;
  }

  abs_tol = device->config.diagnostics.tracking_abs_tolerance_lpm;
  rel_tol =
      (device->config.device.flow_max_lpm - device->config.device.flow_min_lpm) *
      device->config.diagnostics.tracking_relative_tolerance;
  allowed_error = (rel_tol > abs_tol) ? rel_tol : abs_tol;

  error = device->status.target_flow_lpm - device->status.filtered_feedback_flow_lpm;
  if (error < 0.0f)
  {
    error = -error;
  }

  return error <= allowed_error;
}

mfc_result_t MFC_Init(void)
{
  GPIO_PinState off_level;

  MFC_AnalogHwInit();

  for (mfc_channel_t channel = Ar_MFC; channel < MFC_CHANNEL_COUNT; channel++)
  {
    mfc_device_t *device = &g_mfc[channel];

    device->status.target_flow_lpm = device->config.device.flow_min_lpm;
    device->status.feedback_flow_lpm = 0.0f;
    device->status.filtered_feedback_flow_lpm = 0.0f;
    device->status.target_voltage_v = 0.0f;
    device->status.feedback_voltage_v = 0.0f;
    device->status.dac_code = MFC_FlowToDacCode(&device->config, device->config.device.flow_min_lpm);
    device->status.adc_raw = 0U;
    device->status.power_state = MFC_POWER_OFF;
    device->status.tracking_ok = false;
    device->status.feedback_valid = false;
    device->status.feedback_filter_ready = false;
    device->status.fault = false;

    MFC_WriteDacCode(device->config.dac_hw.dac_channel, device->status.dac_code);

    off_level = MFC_GetPowerOffLevel(&device->config.power_hw);
    HAL_GPIO_WritePin(device->config.power_hw.gpio_port,
                      device->config.power_hw.gpio_pin,
                      off_level);

    if ((device->config.channel_enable == MFC_CHANNEL_ENABLED) &&
        !MFC_ValidateConfig(&device->config))
    {
      device->status.power_state = MFC_POWER_FAULT;
      device->status.fault = true;
      return MFC_RESULT_INVALID_CONFIG;
    }
  }

  return MFC_RESULT_OK;
}

bool MFC_IsChannelEnabled(mfc_channel_t channel)
{
  return (channel < MFC_CHANNEL_COUNT) &&
         (g_mfc[channel].config.channel_enable == MFC_CHANNEL_ENABLED);
}

mfc_result_t MFC_PowerOn(mfc_channel_t channel)
{
  mfc_result_t result = MFC_CheckChannel(channel);
  mfc_device_t *device;

  if (result != MFC_RESULT_OK)
  {
    return result;
  }

  device = &g_mfc[channel];
  if (!MFC_ValidateConfig(&device->config))
  {
    device->status.fault = true;
    device->status.power_state = MFC_POWER_FAULT;
    return MFC_RESULT_INVALID_CONFIG;
  }

  (void)MFC_SetFlow(channel, device->config.device.flow_min_lpm);
  device->status.power_state = MFC_POWER_STARTING;
  HAL_GPIO_WritePin(device->config.power_hw.gpio_port,
                    device->config.power_hw.gpio_pin,
                    device->config.power_hw.power_active_level);
  HAL_Delay(device->config.power_hw.power_on_delay_ms);
  device->status.power_state = MFC_POWER_ON;
  device->status.feedback_valid = false;
  device->status.tracking_ok = false;
  device->status.fault = false;

  return MFC_RESULT_OK;
}

mfc_result_t MFC_PowerOff(mfc_channel_t channel)
{
  mfc_result_t result = MFC_CheckChannel(channel);
  mfc_device_t *device;
  GPIO_PinState off_level;

  if (result != MFC_RESULT_OK)
  {
    return result;
  }

  device = &g_mfc[channel];
  device->status.power_state = MFC_POWER_STOPPING;
  (void)MFC_SetFlow(channel, device->config.device.flow_min_lpm);
  HAL_Delay(device->config.power_hw.power_off_delay_ms);
  off_level = MFC_GetPowerOffLevel(&device->config.power_hw);
  HAL_GPIO_WritePin(device->config.power_hw.gpio_port,
                    device->config.power_hw.gpio_pin,
                    off_level);
  device->status.power_state = MFC_POWER_OFF;
  device->status.feedback_valid = false;
  device->status.tracking_ok = false;

  return MFC_RESULT_OK;
}

bool MFC_IsPowered(mfc_channel_t channel)
{
  return (channel < MFC_CHANNEL_COUNT) &&
         (g_mfc[channel].status.power_state == MFC_POWER_ON);
}

mfc_power_state_t MFC_GetPowerState(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return MFC_POWER_FAULT;
  }

  return g_mfc[channel].status.power_state;
}

mfc_result_t MFC_SetFullScaleFlow(mfc_channel_t channel, float full_scale_flow_lpm)
{
  mfc_result_t result = MFC_CheckChannel(channel);

  if (result != MFC_RESULT_OK)
  {
    return result;
  }

  if (full_scale_flow_lpm <= g_mfc[channel].config.device.flow_min_lpm)
  {
    return MFC_RESULT_INVALID_CONFIG;
  }

  g_mfc[channel].config.device.flow_max_lpm = full_scale_flow_lpm;
  return MFC_SetFlow(channel, g_mfc[channel].status.target_flow_lpm);
}

float MFC_GetFullScaleFlow(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return g_mfc[channel].config.device.flow_max_lpm;
}

mfc_result_t MFC_SetFlow(mfc_channel_t channel, float target_flow_lpm)
{
  mfc_result_t result = MFC_CheckChannel(channel);
  mfc_device_t *device;
  uint16_t dac_code;

  if (result != MFC_RESULT_OK)
  {
    return result;
  }

  device = &g_mfc[channel];

  target_flow_lpm = MFC_ClampFloat(target_flow_lpm,
                                   device->config.device.flow_min_lpm,
                                   device->config.device.flow_max_lpm);

  if ((device->status.power_state != MFC_POWER_ON) &&
      (target_flow_lpm > device->config.device.flow_min_lpm))
  {
    return MFC_RESULT_NOT_POWERED;
  }

  if (!MFC_ValidateConfig(&device->config))
  {
    return MFC_RESULT_INVALID_CONFIG;
  }

  dac_code = MFC_FlowToDacCode(&device->config, target_flow_lpm);
  MFC_WriteDacCode(device->config.dac_hw.dac_channel, dac_code);

  device->status.target_flow_lpm = target_flow_lpm;
  device->status.dac_code = dac_code;
  device->status.target_voltage_v = MFC_DacCodeToInterfaceVoltage(&device->config, dac_code);

  return MFC_RESULT_OK;
}

mfc_result_t MFC_UpdateFeedback(mfc_channel_t channel)
{
  mfc_result_t result = MFC_CheckChannel(channel);
  mfc_device_t *device;
  uint16_t adc_raw;
  float feedback_voltage_v;
  float feedback_flow_lpm;

  if (result != MFC_RESULT_OK)
  {
    return result;
  }

  device = &g_mfc[channel];

  if (!MFC_ReadAdcRaw(device->config.adc_hw.adc_channel, &adc_raw))
  {
    device->status.feedback_valid = false;
    return MFC_RESULT_HAL_ERROR;
  }

  feedback_flow_lpm = MFC_AdcCodeToFlow(&device->config, adc_raw, &feedback_voltage_v);

  device->status.adc_raw = adc_raw;
  device->status.feedback_voltage_v = feedback_voltage_v;
  device->status.feedback_flow_lpm = feedback_flow_lpm;
  if (!device->status.feedback_filter_ready)
  {
    device->status.filtered_feedback_flow_lpm = feedback_flow_lpm;
    device->status.feedback_filter_ready = true;
  }
  else
  {
    device->status.filtered_feedback_flow_lpm =
        device->status.filtered_feedback_flow_lpm +
        MFC_FILTER_ALPHA * (feedback_flow_lpm - device->status.filtered_feedback_flow_lpm);
  }
  device->status.feedback_valid = (device->status.power_state == MFC_POWER_ON);
  device->status.tracking_ok = MFC_CheckTracking(device);

  return MFC_RESULT_OK;
}

float MFC_GetTargetFlow(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return g_mfc[channel].status.target_flow_lpm;
}

float MFC_GetInstantFeedbackFlow(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return g_mfc[channel].status.feedback_flow_lpm;
}

float MFC_GetFeedbackFlow(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return g_mfc[channel].status.filtered_feedback_flow_lpm;
}

float MFC_GetTargetVoltage(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return g_mfc[channel].status.target_voltage_v;
}

float MFC_GetFeedbackVoltage(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0.0f;
  }

  return g_mfc[channel].status.feedback_voltage_v;
}

uint16_t MFC_GetAdcRaw(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0U;
  }

  return g_mfc[channel].status.adc_raw;
}

uint16_t MFC_GetDacCode(mfc_channel_t channel)
{
  if (channel >= MFC_CHANNEL_COUNT)
  {
    return 0U;
  }

  return g_mfc[channel].status.dac_code;
}

bool MFC_IsTrackingNormal(mfc_channel_t channel)
{
  return (channel < MFC_CHANNEL_COUNT) && g_mfc[channel].status.tracking_ok;
}

bool MFC_IsFeedbackValid(mfc_channel_t channel)
{
  return (channel < MFC_CHANNEL_COUNT) && g_mfc[channel].status.feedback_valid;
}

bool MFC_HasFault(mfc_channel_t channel)
{
  return (channel < MFC_CHANNEL_COUNT) && g_mfc[channel].status.fault;
}
