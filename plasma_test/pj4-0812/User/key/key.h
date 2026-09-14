#ifndef _KEY_H_
#define _KEY_H_
#include "stm32f10x.h"
#include "usart1.h"

//KEY0
#define key0_port GPIOE
#define key0_pin GPIO_Pin_4
#define key0_EXIT EXTI4_IRQn
#define key0PortSource GPIO_PortSourceGPIOE
#define key0PinSource GPIO_PinSource4
#define key0_EXTILine EXTI_Line4

//KEY1
#define key1_port GPIOE
#define key1_pin GPIO_Pin_3
#define key1_EXIT EXTI3_IRQn
#define key1PortSource GPIO_PortSourceGPIOE
#define key1PinSource GPIO_PinSource3
#define key1_EXTILine EXTI_Line3


//VOL
//VOL-MODE PD5
#define VOLMode_port GPIOD
#define VOLMode_pin GPIO_Pin_5
#define VOLMode_EXIT EXTI9_5_IRQn
#define VOLModePortSource GPIO_PortSourceGPIOD
#define VOLModePinSource GPIO_PinSource5
#define VOLMode_EXTILine EXTI_Line5
//VOL-Contrl PD6
#define VOLContrl_port GPIOD
#define VOLContrl_pin GPIO_Pin_6
#define VOLContrl_EXIT EXTI9_5_IRQn
#define VOLContrlPortSource GPIO_PortSourceGPIOD
#define VOLContrlPinSource GPIO_PinSource6
#define VOLContrl_EXTILine EXTI_Line6

//FLOW
//FLOW-MODE PD1
#define FLOWMode_port GPIOD
#define FLOWMode_pin GPIO_Pin_1
#define FLOWMode_EXIT EXTI1_IRQn
#define FLOWModePortSource GPIO_PortSourceGPIOD
#define FLOWModePinSource GPIO_PinSource1
#define FLOWMode_EXTILine EXTI_Line1
//FLOW-Contrl PD2
#define FLOWContrl_port GPIOD
#define FLOWContrl_pin GPIO_Pin_2
#define FLOWContrl_EXIT EXTI2_IRQn
#define FLOWContrlPortSource GPIO_PortSourceGPIOD
#define FLOWContrlPinSource GPIO_PinSource2
#define FLOWContrl_EXTILine EXTI_Line2


enum Mode{
    CONTINUE,
    KEY_CONTROL
};

extern enum Mode VOLMode;
extern enum Mode FLOWMode;

//NVIC初始化
void NVIC_Configuration1(void);
//按键初始化（包括时钟、中断和引脚）
void key_init(void);
void EXTI_ITConfig(u32 EXTI_LINEn, FunctionalState state);
void KEY_EXTI_Config(u32 EXTI_LINEn, EXTITrigger_TypeDef EXTITrigger_x, FunctionalState state);
void VOLModeChanged(void);
void FLOWModeChanged(void);


#endif

