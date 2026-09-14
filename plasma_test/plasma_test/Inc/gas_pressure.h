#ifndef __GAS_PRESSURE_H__
#define __GAS_PRESSURE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "modbus_rtu.h"
#include <stdbool.h>
#include <stdint.h>

#define GAS_PRESSURE_POLL_MS 500U
#define GAS_PRESSURE_PHASE_MS 250U
#define GAS_PRESSURE_TIMEOUT_MS 300U
#define GAS_PRESSURE_OFFLINE_MS 5000U
#define GAS_PRESSURE_FIRST_REGISTER 0x0002U
#define GAS_PRESSURE_REGISTER_COUNT 5U

typedef enum
{
  GAS_HELIUM = 0,
  GAS_ARGON,
  GAS_COUNT
} Gas_Channel_t;

typedef struct
{
  bool enabled;
  UART_HandleTypeDef *uart;
  uint8_t slave_id;
  uint16_t pressure_register;
  uint16_t register_count;
  float scale;
  float offset;
} GasPressure_Config_t;

typedef struct
{
  float pressure;
  float remaining_percent;
  uint16_t unit;
  uint8_t decimal_places;
  int16_t raw_pressure;
  int16_t raw_range_zero;
  int16_t raw_range_full;
  bool online;
  bool valid;
  bool configured;
  uint32_t last_update_tick;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t exception_count;
} GasPressure_Data_t;

void GasPressure_Init(void);
void GasPressure_Process(void);
bool GasPressure_GetData(Gas_Channel_t channel, GasPressure_Data_t *data);
bool GasPressure_SetConfig(Gas_Channel_t channel, const GasPressure_Config_t *config);
const GasPressure_Config_t *GasPressure_GetConfig(Gas_Channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __GAS_PRESSURE_H__ */
