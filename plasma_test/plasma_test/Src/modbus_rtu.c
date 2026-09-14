#include "modbus_rtu.h"
#include <string.h>

static Modbus_Context_t *g_contexts[MODBUS_RTU_MAX_CONTEXTS] = {0};

static void Modbus_WriteU16BE(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)((value >> 8) & 0xFFU);
  data[1] = (uint8_t)(value & 0xFFU);
}

static uint16_t Modbus_ReadU16BE(const uint8_t *data)
{
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static Modbus_Context_t *Modbus_FindByUart(UART_HandleTypeDef *huart)
{
  for (uint8_t i = 0U; i < MODBUS_RTU_MAX_CONTEXTS; i++)
  {
    if ((g_contexts[i] != NULL) && (g_contexts[i]->uart == huart))
    {
      return g_contexts[i];
    }
  }

  return NULL;
}

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (crc >> 1) ^ 0xA001U;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

Modbus_Result_t Modbus_Init(Modbus_Context_t *ctx, UART_HandleTypeDef *huart)
{
  if ((ctx == NULL) || (huart == NULL))
  {
    return MODBUS_BAD_PARAM;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->uart = huart;
  ctx->result = MODBUS_OK;

  for (uint8_t i = 0U; i < MODBUS_RTU_MAX_CONTEXTS; i++)
  {
    if ((g_contexts[i] == NULL) || (g_contexts[i] == ctx))
    {
      g_contexts[i] = ctx;
      HAL_UART_Receive_IT(ctx->uart, &ctx->rx_byte, 1U);
      return MODBUS_OK;
    }
  }

  return MODBUS_BUSY;
}

Modbus_Result_t Modbus_ReadHoldingRegister(Modbus_Context_t *ctx,
                                           uint8_t slave,
                                           uint16_t address,
                                           uint16_t count,
                                           uint16_t *data,
                                           uint32_t timeout_ms)
{
  uint8_t request[8];
  uint16_t crc;

  if ((ctx == NULL) || (ctx->uart == NULL) || (data == NULL) ||
      (slave == 0U) || (count == 0U) || (count > MODBUS_RTU_MAX_REGISTERS))
  {
    return MODBUS_BAD_PARAM;
  }

  if (ctx->busy)
  {
    return MODBUS_BUSY;
  }

  request[0] = slave;
  request[1] = 0x03U;
  Modbus_WriteU16BE(&request[2], address);
  Modbus_WriteU16BE(&request[4], count);
  crc = Modbus_CRC16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8) & 0xFFU);

  __disable_irq();
  ctx->slave = slave;
  ctx->function = 0x03U;
  ctx->address = address;
  ctx->count = count;
  ctx->dest = data;
  ctx->request_tick = HAL_GetTick();
  ctx->timeout_ms = timeout_ms;
  ctx->expected_length = (uint16_t)(5U + (count * 2U));
  ctx->rx_length = 0U;
  ctx->complete = false;
  ctx->busy = true;
  ctx->result = MODBUS_BUSY;
  __enable_irq();

  if (HAL_UART_Transmit_IT(ctx->uart, request, sizeof(request)) != HAL_OK)
  {
    ctx->busy = false;
    ctx->result = MODBUS_UART_ERROR;
    ctx->stats.uart_error_count++;
    return MODBUS_UART_ERROR;
  }

  ctx->stats.tx_count++;
  return MODBUS_OK;
}

static Modbus_Result_t Modbus_ParseResponse(Modbus_Context_t *ctx)
{
  uint8_t rx[MODBUS_RTU_RX_BUFFER_SIZE];
  uint16_t length;
  uint16_t crc_calc;
  uint16_t crc_recv;

  __disable_irq();
  length = ctx->rx_length;
  if (length > MODBUS_RTU_RX_BUFFER_SIZE)
  {
    length = MODBUS_RTU_RX_BUFFER_SIZE;
  }
  memcpy(rx, ctx->rx_buffer, length);
  ctx->complete = false;
  ctx->busy = false;
  __enable_irq();

  if (length < 5U)
  {
    ctx->result = MODBUS_TIMEOUT;
    ctx->stats.timeout_count++;
    return MODBUS_TIMEOUT;
  }

  crc_calc = Modbus_CRC16(rx, (uint16_t)(length - 2U));
  crc_recv = (uint16_t)rx[length - 2U] | ((uint16_t)rx[length - 1U] << 8);
  if (crc_calc != crc_recv)
  {
    ctx->result = MODBUS_CRC_ERROR;
    ctx->stats.crc_error_count++;
    return MODBUS_CRC_ERROR;
  }

  if ((rx[0] != ctx->slave) || ((rx[1] & 0x7FU) != ctx->function))
  {
    ctx->result = MODBUS_EXCEPTION;
    ctx->stats.exception_count++;
    return MODBUS_EXCEPTION;
  }

  if ((rx[1] & 0x80U) != 0U)
  {
    ctx->result = MODBUS_EXCEPTION;
    ctx->stats.exception_count++;
    return MODBUS_EXCEPTION;
  }

  if ((rx[2] != (uint8_t)(ctx->count * 2U)) || (length != ctx->expected_length))
  {
    ctx->result = MODBUS_EXCEPTION;
    ctx->stats.exception_count++;
    return MODBUS_EXCEPTION;
  }

  for (uint16_t i = 0U; i < ctx->count; i++)
  {
    ctx->dest[i] = Modbus_ReadU16BE(&rx[3U + (i * 2U)]);
  }

  ctx->result = MODBUS_OK;
  ctx->stats.rx_count++;
  return MODBUS_OK;
}

Modbus_Result_t Modbus_Process(Modbus_Context_t *ctx)
{
  if (ctx == NULL)
  {
    return MODBUS_BAD_PARAM;
  }

  if (!ctx->busy)
  {
    return ctx->result;
  }

  if (ctx->complete)
  {
    return Modbus_ParseResponse(ctx);
  }

  if ((HAL_GetTick() - ctx->request_tick) >= ctx->timeout_ms)
  {
    __disable_irq();
    ctx->busy = false;
    ctx->complete = false;
    __enable_irq();
    ctx->result = MODBUS_TIMEOUT;
    ctx->stats.timeout_count++;
    return MODBUS_TIMEOUT;
  }

  return MODBUS_BUSY;
}

bool Modbus_IsBusy(const Modbus_Context_t *ctx)
{
  return (ctx != NULL) && ctx->busy;
}

const Modbus_Stats_t *Modbus_GetStats(const Modbus_Context_t *ctx)
{
  return (ctx != NULL) ? &ctx->stats : NULL;
}

void Modbus_SetDebugLog(Modbus_Context_t *ctx, bool enabled)
{
  if (ctx != NULL)
  {
    ctx->debug_log = enabled;
  }
}

bool Modbus_HandleUartRxCpltCallback(UART_HandleTypeDef *huart)
{
  Modbus_Context_t *ctx = Modbus_FindByUart(huart);

  if (ctx == NULL)
  {
    return false;
  }

  if (ctx->busy && !ctx->complete)
  {
    if (ctx->rx_length < MODBUS_RTU_RX_BUFFER_SIZE)
    {
      ctx->rx_buffer[ctx->rx_length++] = ctx->rx_byte;

      /* Modbus exception responses contain only address, function, code and CRC. */
      if ((ctx->rx_length == 2U) &&
          (ctx->rx_buffer[1] == (uint8_t)(ctx->function | 0x80U)))
      {
        ctx->expected_length = 5U;
      }
    }
    else
    {
      ctx->rx_length = 0U;
      ctx->stats.uart_error_count++;
    }

    if ((ctx->expected_length > 0U) && (ctx->rx_length >= ctx->expected_length))
    {
      ctx->complete = true;
    }
  }

  HAL_UART_Receive_IT(ctx->uart, &ctx->rx_byte, 1U);
  return true;
}

bool Modbus_HandleUartErrorCallback(UART_HandleTypeDef *huart)
{
  Modbus_Context_t *ctx = Modbus_FindByUart(huart);

  if (ctx == NULL)
  {
    return false;
  }

  ctx->stats.uart_error_count++;
  HAL_UART_Receive_IT(ctx->uart, &ctx->rx_byte, 1U);
  return true;
}
