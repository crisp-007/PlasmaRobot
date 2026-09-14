#ifndef __PACKET_COMM_H__
#define __PACKET_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define PACKET_COMM_HEADER       0xFEFFU
#define PACKET_COMM_SIZE         16U
#define PACKET_COMM_STATE_SIZE   28U
#define PACKET_COMM_RX_BUF_SIZE  100U
#define PACKET_COMM_SEND_MS      1000U
#define PACKET_COMM_CMD_SEND_MS  300U

typedef struct
{
  uint16_t header;
  uint8_t EmerStop;
  uint8_t VolRelay;
  uint16_t VolOutValue;
  uint8_t HeFLOWRelay;
  uint16_t HeOutValue;
  uint8_t ArFLOWRelay;
  uint16_t ArOutValue;
  uint32_t footer;
} PacketComm_DataPacket;

typedef enum
{
  PACKET_COMM_OK = 0,
  PACKET_COMM_NO_PACKET,
  PACKET_COMM_BAD_CHECKSUM
} PacketComm_Result;

typedef enum
{
  PACKET_COMM_PORT_LOG = 0,
  PACKET_COMM_PORT_HOST,
  PACKET_COMM_PORT_COUNT
} PacketComm_Port;

extern volatile bool PacketComm_PackReady;
extern PacketComm_DataPacket PacketComm_RecvPacket;
extern PacketComm_DataPacket PacketComm_SendPacket;

void PacketComm_Init(UART_HandleTypeDef *logUart, UART_HandleTypeDef *hostUart);
PacketComm_Result PacketComm_Process(PacketComm_Port port);
void PacketComm_ApplyReceivedPacket(void);
void PacketComm_ProcessHardwareEmergency(void);
void PacketComm_ProcessConnectionIndicator(void);
HAL_StatusTypeDef PacketComm_SendStatus(void);
HAL_StatusTypeDef PacketComm_SendStatePacket(PacketComm_Port port);
uint32_t PacketComm_CalculateChecksum(const uint8_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* __PACKET_COMM_H__ */
