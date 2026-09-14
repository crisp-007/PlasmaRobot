#ifndef _KEY1_H_
#define _KEY1_H_
#include "stm32f10x.h"
#include "usart1.h"

//********************************按键宏*******************************
#define bsp_KeyGroup GPIOE
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



#define ext_KeyGroup GPIOD

//FLOW
//FLOW-MODE PD1
#define emStop_port GPIOD
#define emStop_pin GPIO_Pin_1
#define emStop_EXIT EXTI1_IRQn
#define emStopPortSource GPIO_PortSourceGPIOD
#define emStopPinSource GPIO_PinSource1
#define emStop_EXTILine EXTI_Line1

//急停按钮的当前状态
extern u8 emStop_flag;


//调压器
#define Limit_port GPIOG
//最低电压-限位开关引脚
#define LowLimit_port GPIOG
#define LowLimit_pin GPIO_Pin_10
//最高电压-限位开关引脚
#define HighLimit_port GPIOG
#define HighLimit_pin GPIO_Pin_9





//****************************按键消抖********************************
//按键状态结构体，存储四个变量
typedef struct
{
 	uint8_t KeyLogic;//逻辑状态
	uint8_t KeyPhysic;//物理状态
 	uint8_t KeyONCounts;
 	uint8_t KeyOFFCounts;
}KEY_TypeDef;


//按键状态宏定义
#define    	KEY_ON	   	 	0
#define    	KEY_OFF	   		1
#define    	KEY_HOLD		2
#define		KEY_IDLE		3
#define		KEY_ERROR		10

#define		HOLD_COUNTS			25//间隔20ms->1秒
#define 	SHAKES_COUNTS		3


//按键结构体数组，初始状态都是关闭
static KEY_TypeDef Key[3] =
	{
        {KEY_OFF, KEY_OFF, 0, 0},//emStop_btn
		{KEY_OFF, KEY_OFF, 0, 0},//LowLimit_btn
		{KEY_OFF, KEY_OFF, 0, 0}//HighLimit_btn
    };





//******************************限位开关状态*********************************
#define LimitTrig 1
#define LimitNoTrig 2
#define UseStepModVol 3
#define NotUseStepModVol 0

extern u8 LowLimit_flag;
extern u8 HighLimit_flag;
extern int VolRegInit_Flag;
extern u8 ifUseStep;


#define VOLStart 0
#define VOLStop 1
#define FLOWStart 2
#define FLOWStop 3

//按键初始化（包括时钟、中断和引脚）
void key_init1(void);
void keyGpioInit1(void);


uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void KeyEvent(void);//按键事件扫描并处理


uint8_t GetemStopState(void);


extern u16 Mode_value[5];


#endif

