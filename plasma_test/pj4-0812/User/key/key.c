#include "key.h"

enum Mode VOLMode=CONTINUE;
enum Mode FLOWMode=CONTINUE;

void VOLModeChanged(void)
{
	if(VOLMode==CONTINUE)
	{
		VOLMode=KEY_CONTROL;
		printf("电压模式切换为按键控制\n");
	}else{
		VOLMode=CONTINUE;
		printf("电压模式切换为电位器控制\n");
	}
}

void FLOWModeChanged(void)
{
	if(FLOWMode==CONTINUE)
	{
		FLOWMode=KEY_CONTROL;
		printf("流量模式切换为按键控制\n");
	}else{
		FLOWMode=CONTINUE;
		printf("流量模式切换为电位器控制\n");
	}
}


void NVIC_Configuration1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 嵌套向量中断控制器组选择 */
	/* 提示 NVIC_PriorityGroupConfig() 在整个工程只需要调用一次来配置优先级分组*/
	//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/*******************************************KEY0 NVIC 配置**********************************/
	/* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = key0_EXIT;
	/* 抢断优先级*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	/* 子优先级 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/* 使能中断 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 初始化配置NVIC */
	NVIC_Init(&NVIC_InitStructure);

	/*******************************************KEY1 NVIC 配置**********************************/
	/* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = key1_EXIT;
	/* 初始化配置NVIC */
	NVIC_Init(&NVIC_InitStructure);


	/*******************************************VOL NVIC 配置**********************************/

	NVIC_InitStructure.NVIC_IRQChannel = VOLMode_EXIT;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = VOLContrl_EXIT;
	NVIC_Init(&NVIC_InitStructure);


	/*******************************************FLOW NVIC 配置**********************************/
	NVIC_InitStructure.NVIC_IRQChannel = FLOWMode_EXIT;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = FLOWContrl_EXIT;
	NVIC_Init(&NVIC_InitStructure);
}

void keyGpioInit(void)
{
	/*******************************************各个按键GPIO配置**********************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); // 引脚-板子上的按键
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // 引脚-电压和流量控制按键
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // 外部中断的时钟

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); // 引脚-电压和流量控制按键
	//1、初始化中断引脚
	//1.1、key0引脚配置
	GPIO_InitTypeDef KeyGpio_Init;
	KeyGpio_Init.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
	KeyGpio_Init.GPIO_Pin = key0_pin;
	KeyGpio_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(key0_port, &KeyGpio_Init);
	//1.2、key1引脚配置
	KeyGpio_Init.GPIO_Pin = key1_pin;
	GPIO_Init(key1_port, &KeyGpio_Init);

	//1.3、VOL引脚配置
	//VOL启停引脚配置
	KeyGpio_Init.GPIO_Pin = VOLContrl_pin;
	GPIO_Init(VOLContrl_port, &KeyGpio_Init);
	//VOL模式引脚配置
	KeyGpio_Init.GPIO_Pin = VOLMode_pin;
	GPIO_Init(VOLMode_port, &KeyGpio_Init);

	//1.4、FLOW引脚配置
	//FLOW启停引脚配置
	KeyGpio_Init.GPIO_Pin = FLOWContrl_pin;
	// KeyGpio_Init.GPIO_Mode = GPIO_Mode_IPD; // 上拉输入
	GPIO_Init(FLOWContrl_port, &KeyGpio_Init);
	//FLOW模式引脚配置
	KeyGpio_Init.GPIO_Pin = FLOWMode_pin;
	GPIO_Init(FLOWMode_port, &KeyGpio_Init);


	//2、初始化外部中断线
	EXTI_InitTypeDef KeyExti_Init;
	//2.1、KEY0引脚中断线配置
	GPIO_EXTILineConfig(key0PortSource,key0PinSource);
	KeyExti_Init.EXTI_Line = key0_EXTILine;
	KeyExti_Init.EXTI_Mode = EXTI_Mode_Interrupt;	  // 产生中断
	KeyExti_Init.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿检测
	KeyExti_Init.EXTI_LineCmd = ENABLE;				  // 使能中断
	EXTI_Init(&KeyExti_Init);
	//2.2、KEY1引脚中断线配置
	GPIO_EXTILineConfig(key1PortSource,key1PinSource);
	KeyExti_Init.EXTI_Line = key1_EXTILine;
	KeyExti_Init.EXTI_Mode = EXTI_Mode_Interrupt;	  // 产生中断
	KeyExti_Init.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿检测
	KeyExti_Init.EXTI_LineCmd = ENABLE;				  // 使能中断
	EXTI_Init(&KeyExti_Init);

	//2.3、VOL引脚中断线配置
	//VOL启停
	GPIO_EXTILineConfig(VOLContrlPortSource,VOLContrlPinSource);
	KeyExti_Init.EXTI_Line = VOLContrl_EXTILine;
	KeyExti_Init.EXTI_Mode = EXTI_Mode_Interrupt;	  // 产生中断
	KeyExti_Init.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿检测
	KeyExti_Init.EXTI_LineCmd = ENABLE;				  // 使能中断
	EXTI_Init(&KeyExti_Init);
	//VOL模式
	GPIO_EXTILineConfig(VOLModePortSource,VOLModePinSource);
	KeyExti_Init.EXTI_Line = VOLMode_EXTILine;
	KeyExti_Init.EXTI_Mode = EXTI_Mode_Interrupt;	  // 产生中断
	KeyExti_Init.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿检测
	KeyExti_Init.EXTI_LineCmd = ENABLE;				  // 使能中断
	EXTI_Init(&KeyExti_Init);

	//2.4、FLOW引脚中断线配置
	//FLOW启停
	GPIO_EXTILineConfig(FLOWContrlPortSource,FLOWContrlPinSource);
	KeyExti_Init.EXTI_Line = FLOWContrl_EXTILine;
	EXTI_Init(&KeyExti_Init);
	//FLOW模式
	GPIO_EXTILineConfig(FLOWModePortSource,FLOWModePinSource);
	KeyExti_Init.EXTI_Line = FLOWMode_EXTILine;
	EXTI_Init(&KeyExti_Init);

}

void key_init()
{
	//初始化外部引脚中断
	NVIC_Configuration1();
	keyGpioInit();
}


void EXTI_ITConfig(u32 EXTI_LINEn, FunctionalState state)
{
    if(state)
        EXTI->IMR |= EXTI_LINEn;
    else
        EXTI->IMR &= ~EXTI_LINEn;
}

void KEY_EXTI_Config(u32 EXTI_LINEn, EXTITrigger_TypeDef EXTITrigger_x, FunctionalState state)
{
    EXTI_InitTypeDef EXTI_InitStruct;

    EXTI_InitStruct.EXTI_Line = EXTI_LINEn;
    EXTI_InitStruct.EXTI_Trigger = EXTITrigger_x;
    EXTI_InitStruct.EXTI_LineCmd = state;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_Init(&EXTI_InitStruct);
}
