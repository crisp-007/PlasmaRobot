#include "key1.h"
#include "led.h"
#include "dac.h"
#include "adc.h"
#include "relay.h"
#include "step.h"



// 设备初始状态都为关闭状态
uint8_t emStop_flag=0;
//是否使用步进电机
u8 ifUseStep = NotUseStepModVol;



// 限位器开关状态--初始状态--未触发
u8 LowLimit_flag = LimitNoTrig;
u8 HighLimit_flag = LimitNoTrig;

// 调压器初始化的标志
int VolRegInit_Flag = 1;

// 调压器输出值 0-220
u32 VolRegVal = 0;







void keyGpioInit1(void)
{
	/*******************************************各个按键GPIO配置**********************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); // 引脚-板子上的按键
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // 引脚-电压和流量控制按键
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); // 调压器限位开关引脚

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 外部中断的时钟

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); // 引脚-电压和流量控制按键
	// 1、初始化中断引脚
	// 1.1、key0引脚配置
	GPIO_InitTypeDef KeyGpio_Init;
	KeyGpio_Init.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
	KeyGpio_Init.GPIO_Pin = key0_pin;		// 单独改变
	KeyGpio_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(key0_port, &KeyGpio_Init);
	// 1.2、key1引脚配置
	KeyGpio_Init.GPIO_Pin = key1_pin;
	GPIO_Init(key1_port, &KeyGpio_Init);

	// 1.3、VOL引脚配置
	

	// 1.4、FLOW引脚配置

	// 1.5、急停引脚配置
	KeyGpio_Init.GPIO_Pin = emStop_pin;
	GPIO_Init(emStop_port, &KeyGpio_Init);
	


	// 调压器限位开关
	// 最低电压限位
	KeyGpio_Init.GPIO_Pin = LowLimit_pin;
	GPIO_Init(LowLimit_port, &KeyGpio_Init);
	// 最高电压限位
	KeyGpio_Init.GPIO_Pin = HighLimit_pin;
	GPIO_Init(HighLimit_port, &KeyGpio_Init);
}

void key_init1()
{
	// 初始化外部引脚中断
	keyGpioInit1();
}

/*
 * 函数名：Key_Scan
 * 描述  ：检测是否有按键按下
 * 输入  ：GPIOx：gpio的port
 *		   GPIO_Pin：gpio的pin
 * 输出  ：KEY_OFF、KEY_ON、KEY_HOLD、KEY_IDLE、KEY_ERROR
 */

uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	KEY_TypeDef *KeyTemp;

	// 检查按下的是哪一个按钮
	switch ((uint32_t)GPIOx)
	{
	// 检测端口
	case ((uint32_t)emStop_port):
		switch (GPIO_Pin)
		{
		case emStop_pin:
			KeyTemp = &Key[0];
			break;
		}
		break;

	// 限位开关端口组
	case ((uint32_t)Limit_port):
		switch (GPIO_Pin)
		{
		case LowLimit_pin:
			KeyTemp = &Key[1];
			break;
		case HighLimit_pin:
			KeyTemp = &Key[2];
			break;

			// // port和pin不匹配
			// default:
			// 	printf("error: GPIO port pin not match\r\n");
			// 	return KEY_IDLE;
		}
		break;

	default:
		printf("error: key port do not exist\r\n");
		return KEY_IDLE;
	}

	/* 检测按下、松开、长按 */
	// 对按键的物理状态进行检测
	KeyTemp->KeyPhysic = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);

	// 先检测当前的逻辑状态（x,）
	switch (KeyTemp->KeyLogic)
	{

	case KEY_ON:
		// 当按键的逻辑状态为开时，检测物理状态
		switch (KeyTemp->KeyPhysic)
		{

		// （0，0）将关闭计数清零，并对开启计数累加直到切换至逻辑长按状态
		case KEY_ON:
			// 一直按住的状态，当按住时长到达一定值后，默认为长按
			KeyTemp->KeyOFFCounts = 0;
			KeyTemp->KeyONCounts++;
			if (KeyTemp->KeyONCounts >= HOLD_COUNTS)
			{
				// 按键的累计开启次数大于状态保持次数，长按
				KeyTemp->KeyONCounts = 0;
				KeyTemp->KeyLogic = KEY_HOLD;
				return KEY_HOLD;
			}
			return KEY_IDLE;

		// （0，1）中对关闭计数累加直到切换至逻辑关闭状态
		case KEY_OFF:
			// 按键关闭的瞬间，开始消抖
			KeyTemp->KeyOFFCounts++;
			if (KeyTemp->KeyOFFCounts >= SHAKES_COUNTS)
			{
				KeyTemp->KeyLogic = KEY_OFF;
				KeyTemp->KeyOFFCounts = 0;
				return KEY_OFF;
			}
			return KEY_IDLE;

		default:
			break;
		}

	case KEY_OFF:
		// 当按键的逻辑状态为关时，检测物理状态
		switch (KeyTemp->KeyPhysic)
		{

		// （1，0）中对开启计数累加直到切换至逻辑开启状态
		case KEY_ON:
			// 这个阶段是按键刚开启，有抖动，开始检测抖动次数，到达一定次数后，才默认按键是开启的
			(KeyTemp->KeyONCounts)++;
			if (KeyTemp->KeyONCounts >= SHAKES_COUNTS)
			{
				KeyTemp->KeyLogic = KEY_ON;
				KeyTemp->KeyONCounts = 0;

				return KEY_ON;
			}
			return KEY_IDLE;

		// （1，1）中将开启计数清零
		case KEY_OFF:
			(KeyTemp->KeyONCounts) = 0;
			return KEY_IDLE;
		default:
			break;
		}

	case KEY_HOLD:
		switch (KeyTemp->KeyPhysic)
		{

		// （2，0）对关闭计数清零
		case KEY_ON:
			KeyTemp->KeyOFFCounts = 0;
			return KEY_HOLD;
		// （2，1）对关闭计数累加直到切换至逻辑关闭状态
		case KEY_OFF:
			(KeyTemp->KeyOFFCounts)++;
			if (KeyTemp->KeyOFFCounts >= SHAKES_COUNTS)
			{
				KeyTemp->KeyLogic = KEY_OFF;
				KeyTemp->KeyOFFCounts = 0;
				return KEY_OFF;
			}
			return KEY_IDLE;

		default:
			break;
		}

	default:
		break;
	}

	// 一般不会到这里
	return KEY_ERROR;
}




// void FLOWStateSwitch(void)
// {
// 	if (FLOWWork_flag == FLOWStart)
// 	{
// 		FLOWSetState(FLOWStop);
// 		printf("流量控制器已关闭！\r\n");
// 	}
// 	else
// 	{
// 		FLOWSetState(FLOWStart);
// 		printf("流量控制器已开启！\r\n");
// 	}
// }

//获取急停按钮状态信息
uint8_t GetemStopState(void)
{
	return emStop_flag;
}



void KeyEvent(void)
{
	switch (Key_Scan(ext_KeyGroup, emStop_pin))
	{
	case KEY_ON:
		//急停按钮未被按下，常闭触点接合
		printf("emStop-ON\n");
		emStop_flag=0;
		break;

	case KEY_HOLD:
		// printf("emStop-HOLD\n");
		break;
	case KEY_OFF:
		//急停开关被按下，长闭触点断开
		printf("emStop-OFF\n");
		emStop_flag=1;

		//下位机急停按钮按下处理
		CloseVolDevice();//关闭等离子电源+调压器
		CloseFLOWDevice();//关闭气体供应
		
		//TODO：步进电机归位
		// SendPack();//TODO：上位机处理逻辑存在问题

		break;
	case KEY_ERROR:
		printf("emStop-error\n");
		break;
	default:

		break;
	}
	// if ((VolRegInit_Flag == 1) && (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == Bit_SET))
	// {
	// 	printf("PG10-high\n");
	// 	VolRegInit_Flag = 0;
	// 	VolRegVal = 0;
	// 	ZeroModify(5);
	// }

	// if (ifUseStep==UseStepModVol)
	// {
	// 	// 低压限位开关
	// 	switch (Key_Scan(Limit_port, LowLimit_pin))
	// 	{
	// 	case KEY_ON:
	// 		// 低压限位开关--未触发
	// 		StartStep(StepDownDir_flag);

	// 		LowLimit_flag = LimitNoTrig;
	// 		// printf("LowLimit-ON-NoTrig\n");
	// 		if (LowLimit_flag == LimitNoTrig)
	// 		{
	// 			printf("LowLimit-NoTrig\n");
	// 		}
	// 		break;
	// 	case KEY_HOLD:
	// 		// 低压限位开关--未触发--松弛状态
	// 		//  printf("LowLimit-HOLD\n");
	// 		if (LowLimit_flag == LimitTrig)
	// 		{
	// 			// 防御--如果意外被触发，保持未触发状态
	// 			LowLimit_flag = LimitNoTrig;
	// 		}

	// 		if (VolRegInit_Flag == 1)
	// 			VolRegInit();

	// 		break;
	// 	case KEY_OFF:
	// 		// 1、检测-低压限位开关--触发
	// 		LowLimit_flag = LimitTrig;
	// 		// printf("LowLimit-Trig\n");
	// 		// 2、触发后的处理时间
	// 		StopStep(StepDownDir_flag);

	// 		// 3、输出低压限位器是否被触发的日志
	// 		//  printf("LowLimit-OFF\n");
	// 		if (LowLimit_flag == LimitTrig)
	// 		{
	// 			printf("LowLimit-Trig\n");
	// 			if (VolRegInit_Flag == 1)
	// 			{
	// 				VolRegInit_Flag = 0;
	// 				VolRegVal = 0; // 调压器复位后，输出值为0
	// 				ZeroModify(3);
	// 				printf("调压器初始化完成-%d\n", VolRegInit_Flag);
	// 			}
	// 		}

	// 		break;
	// 	case KEY_ERROR:
	// 		printf("LowLimit-error\n");
	// 		break;
	// 	default:
	// 		// printf("无动作！\n");

	// 		break;
	// 	}

		// 高压限位开关
		// switch (Key_Scan(Limit_port, HighLimit_pin))
		// {
		// case KEY_ON:
		// 	// 高压限位开关--未触发
		// 	//  printf("HighLimit-ON\n");
		// 	StartStep(StepUpDir_flag);
		// 	HighLimit_flag = LimitNoTrig;
		// 	if (HighLimit_flag == LimitNoTrig)
		// 	{
		// 		printf("HighLimit-NoTrig\n");
		// 	}
		// 	break;
		// case KEY_HOLD:
		// 	// 高压限位开关--未触发
		// 	//  printf("HighLimit-HOLD\n");
		// 	if (HighLimit_flag == LimitTrig)
		// 	{
		// 		// 防御--如果意外被触发，保持未触发状态
		// 		HighLimit_flag = LimitNoTrig;
		// 	}
		// 	break;
		// case KEY_OFF:
		// 	// printf("HighLimit-OFF\n");
		// 	// 1、检测-高压限位开关--触发
		// 	HighLimit_flag = LimitTrig;

		// 	// 2、处理-高压限位开关被触发后的处理
		// 	StopStep(StepUpDir_flag);
		// 	// printf("LowLimit-OFF\n");
		// 	if (HighLimit_flag == LimitTrig)
		// 	{
		// 		printf("HighLimit-Trig\n");
		// 	}
		// 	break;
		// case KEY_ERROR:
		// 	printf("HighLimit-error\n");
		// 	break;
		// default:

		// 	break;
		// }
	
}





