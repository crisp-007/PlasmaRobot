#include "gas_pressure.h"
#include "usart.h"
#include <string.h>

typedef struct
{
  GasPressure_Config_t config;
  GasPressure_Data_t data;
  Modbus_Context_t modbus;
  uint16_t raw_registers[MODBUS_RTU_MAX_REGISTERS];
  uint32_t next_poll_tick;
  bool request_pending;
} GasPressure_ChannelState_t;

static GasPressure_ChannelState_t g_gas_pressure[GAS_COUNT] =
{
  [GAS_HELIUM] =
  {
    .config =
    {
      .enabled = true,
      .uart = &he_pressure_uart,
      .slave_id = 1U,
      .pressure_register = GAS_PRESSURE_FIRST_REGISTER,
      .register_count = GAS_PRESSURE_REGISTER_COUNT,
      .scale = 1.0f,
      .offset = 0.0f
    }
  },
  [GAS_ARGON] =
  {
    .config =
    {
      .enabled = true,
      .uart = &ar_pressure_uart,
      .slave_id = 1U,
      .pressure_register = GAS_PRESSURE_FIRST_REGISTER,
      .register_count = GAS_PRESSURE_REGISTER_COUNT,
      .scale = 1.0f,
      .offset = 0.0f
    }
  }
};

static bool GasPressure_ConfigValid(const GasPressure_Config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  if (!config->enabled)
  {
    return false;
  }

  return (config->uart != NULL) &&
         (config->slave_id != 0U) &&
         (config->pressure_register == GAS_PRESSURE_FIRST_REGISTER) &&
         (config->register_count == GAS_PRESSURE_REGISTER_COUNT);
}

static float GasPressure_DecimalDivisor(uint16_t decimal_places)
{
  static const float divisors[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};

  return (decimal_places < (sizeof(divisors) / sizeof(divisors[0])))
      ? divisors[decimal_places]
      : 0.0f;
}

static float GasPressure_UnitToMpa(uint16_t unit)
{
  static const float factors[] =
  {
    1.0f,          /* MPa */
    0.001f,        /* kPa */
    0.000001f,     /* Pa */
    0.1f,          /* bar */
    0.0001f,       /* mbar */
    0.0980665f,    /* kgf/cm2 */
    0.006894757f,  /* PSI */
    0.00980665f,   /* mH2O */
    0.00000980665f,/* mmH2O */
    0.00024908891f,/* inH2O */
    0.0000980665f, /* cmH2O */
    0.133322387f,  /* mHg */
    0.000133322387f,/* mmHg */
    0.003386389f,  /* inHg */
    0.101325f,     /* atm */
    0.000133322368f/* Torr */
  };

  return (unit < (sizeof(factors) / sizeof(factors[0]))) ? factors[unit] : 0.0f;
}

static float GasPressure_CalcRemaining(float pressure,
                                       float range_zero,
                                       float range_full)
{
  float percent;

  if (range_full <= range_zero)
  {
    return 0.0f;
  }

  percent = ((pressure - range_zero) / (range_full - range_zero)) * 100.0f;
  if (percent < 0.0f)
  {
    return 0.0f;
  }
  if (percent > 100.0f)
  {
    return 100.0f;
  }

  return percent;
}

static void GasPressure_ApplyResult(GasPressure_ChannelState_t *state)
{
  float divisor;
  float unit_to_mpa;
  float pressure;
  float range_zero;
  float range_full;

  state->data.unit = state->raw_registers[0];
  state->data.decimal_places = (uint8_t)state->raw_registers[1];
  state->data.raw_pressure = (int16_t)state->raw_registers[2];
  state->data.raw_range_zero = (int16_t)state->raw_registers[3];
  state->data.raw_range_full = (int16_t)state->raw_registers[4];
  state->data.online = true;
  state->data.last_update_tick = HAL_GetTick();

  divisor = GasPressure_DecimalDivisor(state->raw_registers[1]);
  unit_to_mpa = GasPressure_UnitToMpa(state->data.unit);
  if ((divisor <= 0.0f) || (unit_to_mpa <= 0.0f))
  {
    state->data.valid = false;
    return;
  }

  pressure = ((float)state->data.raw_pressure / divisor) * unit_to_mpa;
  range_zero = ((float)state->data.raw_range_zero / divisor) * unit_to_mpa;
  range_full = ((float)state->data.raw_range_full / divisor) * unit_to_mpa;

  pressure = (pressure * state->config.scale) + state->config.offset;
  range_zero = (range_zero * state->config.scale) + state->config.offset;
  range_full = (range_full * state->config.scale) + state->config.offset;

  state->data.pressure = pressure;
  state->data.remaining_percent =
      GasPressure_CalcRemaining(pressure, range_zero, range_full);
  state->data.valid = true;
}

void GasPressure_Init(void)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0U; i < GAS_COUNT; i++)
  {
    GasPressure_ChannelState_t *state = &g_gas_pressure[i];

    memset(&state->data, 0, sizeof(state->data));
    memset(state->raw_registers, 0, sizeof(state->raw_registers));
    state->request_pending = false;
    state->next_poll_tick = now + ((i == GAS_ARGON) ? GAS_PRESSURE_PHASE_MS : 0U);
    state->data.configured = GasPressure_ConfigValid(&state->config);
    (void)Modbus_Init(&state->modbus, state->config.uart);
  }
}

void GasPressure_Process(void)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0U; i < GAS_COUNT; i++)
  {
    GasPressure_ChannelState_t *state = &g_gas_pressure[i];
    Modbus_Result_t result;

    state->data.configured = GasPressure_ConfigValid(&state->config);
    if (!state->data.configured)
    {
      state->data.valid = false;
      state->data.online = false;
      continue;
    }

    result = Modbus_Process(&state->modbus);
    if (state->request_pending && (result != MODBUS_BUSY))
    {
      state->request_pending = false;
      if (result == MODBUS_OK)
      {
        GasPressure_ApplyResult(state);
      }
      else
      {
        const Modbus_Stats_t *stats = Modbus_GetStats(&state->modbus);

        if (stats != NULL)
        {
          state->data.timeout_count = stats->timeout_count;
          state->data.crc_error_count = stats->crc_error_count;
          state->data.exception_count = stats->exception_count;
        }
      }
      state->next_poll_tick = now + GAS_PRESSURE_POLL_MS;
    }

    if ((state->data.last_update_tick == 0U) ||
        ((now - state->data.last_update_tick) > GAS_PRESSURE_OFFLINE_MS))
    {
      state->data.online = false;
    }

    if (!state->request_pending && !Modbus_IsBusy(&state->modbus) &&
        ((int32_t)(now - state->next_poll_tick) >= 0))
    {
      result = Modbus_ReadHoldingRegister(&state->modbus,
                                          state->config.slave_id,
                                          state->config.pressure_register,
                                          state->config.register_count,
                                          state->raw_registers,
                                          GAS_PRESSURE_TIMEOUT_MS);
      if (result == MODBUS_OK)
      {
        state->request_pending = true;
      }
      else
      {
        state->next_poll_tick = now + GAS_PRESSURE_POLL_MS;
      }
    }
  }
}

bool GasPressure_GetData(Gas_Channel_t channel, GasPressure_Data_t *data)
{
  if ((channel >= GAS_COUNT) || (data == NULL))
  {
    return false;
  }

  *data = g_gas_pressure[channel].data;
  return true;
}

bool GasPressure_SetConfig(Gas_Channel_t channel, const GasPressure_Config_t *config)
{
  if ((channel >= GAS_COUNT) || (config == NULL))
  {
    return false;
  }

  g_gas_pressure[channel].config = *config;
  g_gas_pressure[channel].data.configured = GasPressure_ConfigValid(config);
  g_gas_pressure[channel].request_pending = false;
  (void)Modbus_Init(&g_gas_pressure[channel].modbus, config->uart);

  return true;
}

const GasPressure_Config_t *GasPressure_GetConfig(Gas_Channel_t channel)
{
  if (channel >= GAS_COUNT)
  {
    return NULL;
  }

  return &g_gas_pressure[channel].config;
}
