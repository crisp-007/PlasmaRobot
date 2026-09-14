#include "bsp_plasma_feedback.h"
#include <math.h>
#include <string.h>

#define PLASMA_FB_ADC_CHANNEL          6U
#define PLASMA_FB_PROCESS_SAMPLES      (PLASMA_FB_DMA_BUFFER_LENGTH / 2U)
#define PLASMA_FB_TIM_CLOCK_HZ         84000000U
#define PLASMA_FB_ADC_EXTSEL_TIM2_TRGO 6U

static const plasma_feedback_config_t g_plasma_fb_cfg =
{
  .input_min_v = -30.0f,
  .input_max_v = 30.0f,
  .nominal_frequency_hz = 30000.0f,
  .divider_ratio = 1.0f / 18.0f,
  .bias_voltage_v = 1.715f,//1.67
  .adc_vref_v = 3.3f,
  .adc_full_scale_code = 4095U,
  .sample_rate_hz = 600000U,
  .dma_buffer_length = PLASMA_FB_DMA_BUFFER_LENGTH,
  .feedback_to_hv_kv_per_v = 1.0f,
  .gain_cal = 1.15f,//1.0
  .offset_cal_v = 0.0f,
  .bias_cal_v = 0.0f,
  .adc_clip_low_code = 5U,
  .adc_clip_high_code = 4090U,
  .min_valid_amplitude_v = 0.5f,
  .frequency_tolerance_hz = 3000.0f
};

static uint16_t g_plasma_fb_dma_buffer[PLASMA_FB_DMA_BUFFER_LENGTH] = {0};
static volatile bool g_half_ready = false;
static volatile bool g_full_ready = false;
static volatile bool g_running = false;
static bool g_initialized = false;
static plasma_feedback_measurement_t g_measurement = {0};

static bool PlasmaFeedback_ConfigValid(void)
{
  return (g_plasma_fb_cfg.divider_ratio > 0.0f) &&
         (g_plasma_fb_cfg.adc_vref_v > 0.0f) &&
         (g_plasma_fb_cfg.adc_full_scale_code > 0U) &&
         (g_plasma_fb_cfg.sample_rate_hz > 0U) &&
         (g_plasma_fb_cfg.dma_buffer_length == PLASMA_FB_DMA_BUFFER_LENGTH) &&
         (g_plasma_fb_cfg.input_max_v > g_plasma_fb_cfg.input_min_v);
}

static float PlasmaFeedback_AdcRawToPinVoltage(uint16_t raw)
{
  return ((float)raw / (float)g_plasma_fb_cfg.adc_full_scale_code) *
         g_plasma_fb_cfg.adc_vref_v;
}

static float PlasmaFeedback_PinVoltageToFeedback(float pin_v)
{
  float bias = g_plasma_fb_cfg.bias_voltage_v + g_plasma_fb_cfg.bias_cal_v;
  float feedback_v = (pin_v - bias) / g_plasma_fb_cfg.divider_ratio;

  return feedback_v * g_plasma_fb_cfg.gain_cal + g_plasma_fb_cfg.offset_cal_v;
}

static float PlasmaFeedback_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float PlasmaFeedback_CalculateFrequency(const float *samples,
                                               uint32_t count,
                                               float offset_v)
{
  float first_crossing = 0.0f;
  float last_crossing = 0.0f;
  float previous;
  float current;
  uint32_t crossing_count = 0U;

  if ((samples == NULL) || (count < 2U))
  {
    return 0.0f;
  }

  previous = samples[0] - offset_v;
  for (uint32_t i = 1U; i < count; i++)
  {
    current = samples[i] - offset_v;
    if ((previous < 0.0f) && (current >= 0.0f))
    {
      float denominator = current - previous;
      float fraction = 0.0f;
      float crossing;

      if (denominator != 0.0f)
      {
        fraction = -previous / denominator;
      }
      crossing = (float)(i - 1U) + fraction;

      if (crossing_count == 0U)
      {
        first_crossing = crossing;
      }
      last_crossing = crossing;
      crossing_count++;
    }
    previous = current;
  }

  if (crossing_count < 2U)
  {
    return 0.0f;
  }

  return ((float)g_plasma_fb_cfg.sample_rate_hz * (float)(crossing_count - 1U)) /
         (last_crossing - first_crossing);
}

static plasma_feedback_result_t PlasmaFeedback_AnalyzeWindow(const uint16_t *raw,
                                                             uint32_t count)
{
  static float feedback_samples[PLASMA_FB_PROCESS_SAMPLES];
  plasma_feedback_measurement_t next = {0};
  float sum = 0.0f;
  float square_sum = 0.0f;

  if ((raw == NULL) || (count == 0U) || (count > PLASMA_FB_PROCESS_SAMPLES))
  {
    return PLASMA_FB_INVALID_CONFIG;
  }

  next.adc_min_raw = 0xFFFFU;
  next.adc_max_raw = 0U;
  next.feedback_min_v = 1000000.0f;
  next.feedback_max_v = -1000000.0f;
  next.sample_count = count;

  for (uint32_t i = 0U; i < count; i++)
  {
    float pin_v;
    float feedback_v;

    if (raw[i] < next.adc_min_raw)
    {
      next.adc_min_raw = raw[i];
    }
    if (raw[i] > next.adc_max_raw)
    {
      next.adc_max_raw = raw[i];
    }
    if ((raw[i] <= g_plasma_fb_cfg.adc_clip_low_code) ||
        (raw[i] >= g_plasma_fb_cfg.adc_clip_high_code))
    {
      next.clipped = true;
    }

    pin_v = PlasmaFeedback_AdcRawToPinVoltage(raw[i]);
    feedback_v = PlasmaFeedback_PinVoltageToFeedback(pin_v);
    feedback_samples[i] = feedback_v;
    sum += feedback_v;

    if (feedback_v < next.feedback_min_v)
    {
      next.feedback_min_v = feedback_v;
    }
    if (feedback_v > next.feedback_max_v)
    {
      next.feedback_max_v = feedback_v;
    }
  }

  next.adc_min_voltage_v = PlasmaFeedback_AdcRawToPinVoltage(next.adc_min_raw);
  next.adc_max_voltage_v = PlasmaFeedback_AdcRawToPinVoltage(next.adc_max_raw);
  next.feedback_offset_v = sum / (float)count;
  next.feedback_vpp_v = next.feedback_max_v - next.feedback_min_v;
  next.feedback_peak_v = next.feedback_vpp_v / 2.0f;

  for (uint32_t i = 0U; i < count; i++)
  {
    float ac_v = feedback_samples[i] - next.feedback_offset_v;
    square_sum += ac_v * ac_v;
  }

  next.feedback_rms_v = sqrtf(square_sum / (float)count);
  next.feedback_frequency_hz =
      PlasmaFeedback_CalculateFrequency(feedback_samples, count, next.feedback_offset_v);
  next.signal_valid = (next.feedback_vpp_v >= g_plasma_fb_cfg.min_valid_amplitude_v);
  next.frequency_valid =
      next.signal_valid &&
      (PlasmaFeedback_Abs(next.feedback_frequency_hz - g_plasma_fb_cfg.nominal_frequency_hz) <=
       g_plasma_fb_cfg.frequency_tolerance_hz);
  next.hv_peak_kv = next.feedback_peak_v * g_plasma_fb_cfg.feedback_to_hv_kv_per_v;
  next.hv_vpp_kv = next.feedback_vpp_v * g_plasma_fb_cfg.feedback_to_hv_kv_per_v;
  next.hv_rms_kv = next.feedback_rms_v * g_plasma_fb_cfg.feedback_to_hv_kv_per_v;
  next.update_counter = g_measurement.update_counter + 1U;

  g_measurement = next;

  if (next.clipped)
  {
    return PLASMA_FB_CLIPPED;
  }
  if (!next.signal_valid)
  {
    return PLASMA_FB_NO_SIGNAL;
  }
  if (!next.frequency_valid)
  {
    return PLASMA_FB_FREQUENCY_ABNORMAL;
  }

  return PLASMA_FB_OK;
}

static void PlasmaFeedback_HwInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  DMA2_Stream0->CR &= ~DMA_SxCR_EN;
  while ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0U)
  {
  }
  DMA2->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
                DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
  DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
  DMA2_Stream0->M0AR = (uint32_t)g_plasma_fb_dma_buffer;
  DMA2_Stream0->NDTR = PLASMA_FB_DMA_BUFFER_LENGTH;
  DMA2_Stream0->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 |
                     DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE |
                     DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  DMA2_Stream0->FCR = 0U;

  ADC->CCR &= ~ADC_CCR_ADCPRE;
  ADC->CCR |= ADC_CCR_ADCPRE_0;
  ADC1->CR1 = 0U;
  ADC1->CR2 = 0U;
  ADC1->SQR1 = 0U;
  ADC1->SQR3 = PLASMA_FB_ADC_CHANNEL;
  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP6);
  ADC1->SMPR2 |= ADC_SMPR2_SMP6_0;
  ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS |
              (PLASMA_FB_ADC_EXTSEL_TIM2_TRGO << ADC_CR2_EXTSEL_Pos) |
              ADC_CR2_EXTEN_0 | ADC_CR2_ADON;

  TIM2->CR1 = 0U;
  TIM2->PSC = 0U;
  TIM2->ARR = (PLASMA_FB_TIM_CLOCK_HZ / g_plasma_fb_cfg.sample_rate_hz) - 1U;
  TIM2->CR2 &= ~TIM_CR2_MMS;
  TIM2->CR2 |= TIM_CR2_MMS_1;
  TIM2->EGR = TIM_EGR_UG;

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

plasma_feedback_result_t PlasmaFeedback_Init(void)
{
  if (!PlasmaFeedback_ConfigValid())
  {
    return PLASMA_FB_INVALID_CONFIG;
  }

  PlasmaFeedback_HwInit();
  memset(g_plasma_fb_dma_buffer, 0, sizeof(g_plasma_fb_dma_buffer));
  memset(&g_measurement, 0, sizeof(g_measurement));
  g_half_ready = false;
  g_full_ready = false;
  g_running = false;
  g_initialized = true;

  return PLASMA_FB_OK;
}

plasma_feedback_result_t PlasmaFeedback_Start(void)
{
  if (!g_initialized)
  {
    return PLASMA_FB_NOT_INITIALIZED;
  }
  if (g_running)
  {
    return PLASMA_FB_ALREADY_RUNNING;
  }

  memset(g_plasma_fb_dma_buffer, 0, sizeof(g_plasma_fb_dma_buffer));
  memset(&g_measurement, 0, sizeof(g_measurement));
  g_half_ready = false;
  g_full_ready = false;

  DMA2_Stream0->CR &= ~DMA_SxCR_EN;
  while ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0U)
  {
  }
  DMA2_Stream0->NDTR = PLASMA_FB_DMA_BUFFER_LENGTH;
  DMA2->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
                DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
  ADC1->SR = 0U;
  ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS;
  DMA2_Stream0->CR |= DMA_SxCR_EN;
  TIM2->CNT = 0U;
  TIM2->CR1 |= TIM_CR1_CEN;
  g_running = true;

  return PLASMA_FB_OK;
}

plasma_feedback_result_t PlasmaFeedback_Stop(void)
{
  if (!g_initialized)
  {
    return PLASMA_FB_NOT_INITIALIZED;
  }
  if (!g_running)
  {
    return PLASMA_FB_NOT_RUNNING;
  }

  TIM2->CR1 &= ~TIM_CR1_CEN;
  ADC1->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_DDS);
  DMA2_Stream0->CR &= ~DMA_SxCR_EN;
  while ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0U)
  {
  }
  g_running = false;

  return PLASMA_FB_OK;
}

bool PlasmaFeedback_IsRunning(void)
{
  return g_running;
}

plasma_feedback_result_t PlasmaFeedback_Process(void)
{
  const uint16_t *window = NULL;

  if (!g_initialized)
  {
    return PLASMA_FB_NOT_INITIALIZED;
  }
  if (!g_running)
  {
    return PLASMA_FB_NOT_RUNNING;
  }

  __disable_irq();
  if (g_half_ready)
  {
    g_half_ready = false;
    window = &g_plasma_fb_dma_buffer[0];
  }
  else if (g_full_ready)
  {
    g_full_ready = false;
    window = &g_plasma_fb_dma_buffer[PLASMA_FB_PROCESS_SAMPLES];
  }
  __enable_irq();

  if (window == NULL)
  {
    return PLASMA_FB_OK;
  }

  return PlasmaFeedback_AnalyzeWindow(window, PLASMA_FB_PROCESS_SAMPLES);
}

bool PlasmaFeedback_GetMeasurement(plasma_feedback_measurement_t *measurement)
{
  if (measurement == NULL)
  {
    return false;
  }

  __disable_irq();
  *measurement = g_measurement;
  __enable_irq();

  return (g_measurement.update_counter > 0U);
}

const uint16_t *PlasmaFeedback_GetRawBuffer(uint32_t *sample_count)
{
  if (sample_count != NULL)
  {
    *sample_count = PLASMA_FB_DMA_BUFFER_LENGTH;
  }

  return g_plasma_fb_dma_buffer;
}

void PlasmaFeedback_OnDmaHalfComplete(void)
{
  g_half_ready = true;
}

void PlasmaFeedback_OnDmaComplete(void)
{
  g_full_ready = true;
}
