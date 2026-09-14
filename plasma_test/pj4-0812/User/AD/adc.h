#ifndef _ADC_H_
#define _ADC_H_

#include "stm32f10x.h"

//ADC配置宏
#define ADCx ADC1
#define ADC_PORT GPIOC
#define ADC_CLK  RCC_APB2Periph_GPIOC
//1、等离子电压采集
#define ADC_VOL_PIN GPIO_Pin_2
#define VOL_CHANNEL ADC_Channel_12

//2、氦气流量采集
#define ADC_HE_FLOW_PIN GPIO_Pin_1
#define HE_FLOW_CHANNEL ADC_Channel_11

//3、氩气流量采集
#define ADC_AR_FLOW_PIN GPIO_Pin_3
#define AR_FLOW_CHANNEL ADC_Channel_13

// ADC通道索引定义（对应ADC_ConvertedValue数组索引）
#define VOL_ADC_INDEX     0  // 等离子电压采集通道索引
#define HE_FLOW_ADC_INDEX 1  // 氦气流量采集通道索引
#define AR_FLOW_ADC_INDEX 2  // 氩气流量采集通道索引

// ADC 中断相关宏定义
#define ADC_IRQ ADC1_2_IRQn
#define ADC_IRQHandler ADC1_2_IRQHandler

//DMA
#define ADC_DMA_CHANNEL DMA1_Channel1//ADC1 对应 DMA1 通道 1，ADC3 对应 DMA2 通道 5，ADC2 没有 DMA 功能
#define NOFCHANEL 3 //转换通道个数

// extern u16 ADC_ConvertedValue;
// extern float ADC_ConvertedValueLocal;
extern u16 ADC_ConvertedValue[NOFCHANEL];
extern float ADC_ConvertedValueLocal[NOFCHANEL];

void ADC_GPIO_Init(void);
void ADC_Config(void);
void ADC_NVICConfig(void);
void ADC_Init1(void);


#endif
