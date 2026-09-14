#ifndef _USART1_H_
#define _USART1_H_
#include "stm32f10x.h"
#include "led.h"
#include <stdio.h>

#define BUFSIZE 50

#define Usartx USART2
#define Usart_Port GPIOA
#define Usart_Tx GPIO_Pin_2
#define Usart_Rx GPIO_Pin_3
#define Usart1_Rate 115200 
#define USART_IRQn USART2_IRQn


#define CmdUsart USART1
#define CmdUsart_Port GPIOA
#define CmdUsart_Tx GPIO_Pin_9
#define CmdUsart_Rx GPIO_Pin_10
#define CmdUsart_Rate 115200
#define CmdUSART_IRQn USART1_IRQn




extern char RecvBuf[BUFSIZE];
extern int RecvBuf_Index;
extern int bufReady; 
extern int ifPrintf;
//串口引脚配置
void Usart1_Init(void);
void CmdUsart_Init(void);
//发送一个字符
void Usart1_SendByte(uint8_t strData);
//发送一个字符串
void Usart1_SendStr(char *strData);
//NVIC初始化
void NVIC_Configuration(void);

int fputc(int ch,FILE *f);

int fgetc(FILE *f);
void USART_SendOneByte(uint16_t Data);

void USART_SendU8(uint8_t data);
void USART_SendU16(uint16_t data);
void USART_SendU32(uint32_t data);

#endif

