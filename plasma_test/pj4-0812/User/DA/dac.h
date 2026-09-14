#ifndef _DAC_H_
#define _DAC_H_

#include "stm32f10x.h"


//DAC配置宏
#define DAC_Port GPIOA


#define HE_FLOW_OUT_Pin GPIO_Pin_4  // 氦气流量控制输出
#define AR_FLOW_OUT_Pin GPIO_Pin_5  // 氩气流量控制输出

// DAC通道定义
#define HE_FLOW_DAC_CHANNEL DAC_Channel_1  // 氦气流量DAC通道
#define AR_FLOW_DAC_CHANNEL DAC_Channel_2  // 氩气流量DAC通道


extern float DAC_CurValue_HeFlow;
extern float DAC_CurValue_ArFlow;

extern u16 heFlowKeyValue;
extern u16 arFlowKeyValue;

void DAC_GpioInit(void);
void DAC_Config(void);
void DAC_Init1(void);
void Set_HeFlow_Value(u16 value);
void Set_ArFlow_Value(u16 value);
uint16_t Get_HeFlow_Value(void);
uint16_t Get_ArFlow_Value(void);
void Dac1_Set_Vol(u16 vol);
void Dac1_Init(void);





#endif




