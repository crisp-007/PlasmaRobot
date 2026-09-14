#ifndef __BSP_PLASMA_FEEDBACK_H__
#define __BSP_PLASMA_FEEDBACK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define PLASMA_FB_DMA_BUFFER_LENGTH 400U

typedef struct
{
  float input_min_v;
  float input_max_v;
  float nominal_frequency_hz;
  float divider_ratio;
  float bias_voltage_v;
  float adc_vref_v;
  uint16_t adc_full_scale_code;
  uint32_t sample_rate_hz;
  uint32_t dma_buffer_length;
  float feedback_to_hv_kv_per_v;
  float gain_cal;
  float offset_cal_v;
  float bias_cal_v;
  uint16_t adc_clip_low_code;
  uint16_t adc_clip_high_code;
  float min_valid_amplitude_v;
  float frequency_tolerance_hz;
} plasma_feedback_config_t;

typedef struct
{
  uint16_t adc_min_raw;
  uint16_t adc_max_raw;
  float adc_min_voltage_v;
  float adc_max_voltage_v;
  float feedback_min_v;
  float feedback_max_v;
  float feedback_vpp_v;
  float feedback_peak_v;
  float feedback_offset_v;
  float feedback_rms_v;
  float feedback_frequency_hz;
  float hv_peak_kv;
  float hv_vpp_kv;
  float hv_rms_kv;
  bool signal_valid;
  bool clipped;
  bool frequency_valid;
  uint32_t sample_count;
  uint32_t update_counter;
} plasma_feedback_measurement_t;

typedef enum
{
  PLASMA_FB_OK = 0,
  PLASMA_FB_NOT_INITIALIZED,
  PLASMA_FB_ALREADY_RUNNING,
  PLASMA_FB_NOT_RUNNING,
  PLASMA_FB_INVALID_CONFIG,
  PLASMA_FB_HAL_ERROR,
  PLASMA_FB_NO_SIGNAL,
  PLASMA_FB_CLIPPED,
  PLASMA_FB_FREQUENCY_ABNORMAL
} plasma_feedback_result_t;

plasma_feedback_result_t PlasmaFeedback_Init(void);
plasma_feedback_result_t PlasmaFeedback_Start(void);
plasma_feedback_result_t PlasmaFeedback_Stop(void);
bool PlasmaFeedback_IsRunning(void);
plasma_feedback_result_t PlasmaFeedback_Process(void);
bool PlasmaFeedback_GetMeasurement(plasma_feedback_measurement_t *measurement);
const uint16_t *PlasmaFeedback_GetRawBuffer(uint32_t *sample_count);
void PlasmaFeedback_OnDmaHalfComplete(void);
void PlasmaFeedback_OnDmaComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PLASMA_FEEDBACK_H__ */
