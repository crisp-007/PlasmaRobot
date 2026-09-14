#ifndef _CMD_H_
#define _CMD_H_

#include "stm32f10x.h"
#include "string.h"
#include <stdbool.h>
#include "usart1.h"
#include "systic.h"
#include "relay.h"
#include "dac.h"
#include "adc.h"
#include "key1.h"

//cmd模块功能:解析上位机串口发送过来的命令行

#define HEADER 0xFEFF //包头
#define FOOTER 0xFE //包尾

// //定义数据包的结构
// typedef struct {
//     uint16_t header;//包头

//     uint8_t EmerStop;//紧急停止

//     uint8_t VolRelay;
//     uint8_t VolModRelay;
//     uint8_t FlowRelay;

//     uint16_t VolState;
//     uint16_t VolSetState;
//     uint16_t FlowState;
//     uint32_t footer;
// } DataPacket;

//定义数据包的结构
typedef struct {
    uint16_t header;//包头

    uint8_t EmerStop;//紧急停止

    //等离子电源相关
    uint8_t VolRelay;//高4位为等离子电源继电器,低4位为调压器继电器
    uint16_t VolOutValue;//等离子电源输出值

    //氦气
    uint8_t HeFLOWRelay;//高4位控制流量计的开关，低4位控制电磁阀的开关
    uint16_t HeOutValue;//氦气流量器的输出值设定
    //氩气
    uint8_t ArFLOWRelay;//高4位控制流量计的开关，低4位控制电磁阀的开关
    uint16_t ArOutValue;//氩气流量器的输出值设定

    uint32_t footer;
} DataPacket;

// 串口接收缓冲区
#define RX_BUFFER_SIZE 100 //定义接收缓冲区大小
extern uint8_t recv_buf[RX_BUFFER_SIZE];  // USART1命令串口缓冲区
extern uint8_t log_buf[RX_BUFFER_SIZE];   // USART2日志串口缓冲区
extern uint8_t cmd_buf[RX_BUFFER_SIZE];
extern uint16_t rxIndex;      // USART1索引
extern uint16_t logIndex;     // USART2索引
extern bool PackReady;

extern bool ifHead;
extern bool ifComplete;

extern DataPacket SendPacket;//要发送的数据包
extern DataPacket RecvPacket;//要接收的数据包


u8 SendPack(void);//发送数据包

void DeviceStateLog(void);//打印数据包内容-设备状态

void DealPortData(bool iflog);//接收串口数据
u8 CmdDeal(bool ready);


//数据包处理
// void ProcessDataPacket(DataPacket* packet) 
// {
//     //处理数据包的内容
//     printf("EmerStop: %d\n", packet->EmerStop);
//     printf("VolRelay: %d\n", packet->VolRelay);
//     printf("FlowRelay: %d\n", packet->FlowRelay);
//     printf("VolState: %u\n", packet->VolState);
//     printf("VolSetState: %u\n", packet->VolSetState);
//     printf("FlowState: %u\n", packet->FlowState);
// }

// void 

// rxBuffer[index++] = USART_ReceiveData(USART1);

//         if (index == sizeof(rxBuffer)) 
//         {
//             if () 
//             {
//                 DataPacket packet;
//                 packet.header[0] = rxBuffer[0];
//                 packet.header[1] = rxBuffer[1];
//                 packet.EmerStop = (rxBuffer[2] != 0);
//                 packet.VolRelay = (rxBuffer[3] != 0);
//                 packet.FlowRelay = (rxBuffer[4] != 0);
//                 packet.VolState = (rxBuffer[5] << 8) | rxBuffer[6];
//                 packet.VolSetState = (rxBuffer[7] << 8) | rxBuffer[8];
//                 packet.FlowState = (rxBuffer[9] << 8) | rxBuffer[10];
//                 packet.footer = rxBuffer[11];
                
//                 ProcessDataPacket(&packet);
//             }
            
//             index = 0; // 重置索引
//         }

#endif
