#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define MODBUS_RTU_MAX_CONTEXTS 2U
#define MODBUS_RTU_MAX_REGISTERS 8U
#define MODBUS_RTU_RX_BUFFER_SIZE 64U

typedef enum
{
  MODBUS_OK = 0,
  MODBUS_BUSY,
  MODBUS_NOT_FOUND,
  MODBUS_BAD_PARAM,
  MODBUS_TIMEOUT,
  MODBUS_CRC_ERROR,
  MODBUS_EXCEPTION,
  MODBUS_UART_ERROR
} Modbus_Result_t;

typedef struct
{
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t exception_count;
  uint32_t uart_error_count;
} Modbus_Stats_t;

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t slave;
  uint8_t function;
  uint16_t address;
  uint16_t count;
  uint16_t *dest;
  uint32_t request_tick;
  uint32_t timeout_ms;
  uint16_t expected_length;
  volatile uint16_t rx_length;
  uint8_t rx_byte;
  uint8_t rx_buffer[MODBUS_RTU_RX_BUFFER_SIZE];
  volatile bool busy;
  volatile bool complete;
  bool debug_log;
  Modbus_Result_t result;
  Modbus_Stats_t stats;
} Modbus_Context_t;

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length);
Modbus_Result_t Modbus_Init(Modbus_Context_t *ctx, UART_HandleTypeDef *huart);
Modbus_Result_t Modbus_ReadHoldingRegister(Modbus_Context_t *ctx,
                                           uint8_t slave,
                                           uint16_t address,
                                           uint16_t count,
                                           uint16_t *data,
                                           uint32_t timeout_ms);
Modbus_Result_t Modbus_Process(Modbus_Context_t *ctx);
bool Modbus_IsBusy(const Modbus_Context_t *ctx);
const Modbus_Stats_t *Modbus_GetStats(const Modbus_Context_t *ctx);
void Modbus_SetDebugLog(Modbus_Context_t *ctx, bool enabled);
bool Modbus_HandleUartRxCpltCallback(UART_HandleTypeDef *huart);
bool Modbus_HandleUartErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_RTU_H__ */
