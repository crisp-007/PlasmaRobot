/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
 * @author  MCD Application Team
 * @version V3.5.0
 * @date    08-April-2011
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usart1.h"
#include "key1.h"
#include "adc.h"
#include "systic.h"
#include "step.h"
/** @addtogroup STM32F10x_StdPeriph_Template
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
	if (timeflag != 0)
	{
		timeflag--;
	}
}

// KEY1中断服务函数
void EXTI3_IRQHandler(void)
{
	// 确保是否产生了 EXTI Line 中断
	if (EXTI_GetITStatus(key1_EXTILine) != RESET) // key1_EXTILine key1_EXIT
	{
		// LED2 取反
		printf("进入到按键1中断服务函数里面\n");
		Led1_Change();
		// 清除中断标志位
		EXTI_ClearITPendingBit(key1_EXTILine);
	}
}
// KEY0中断服务函数
void EXTI4_IRQHandler(void)
{
	// 确保是否产生了 EXTI Line 中断
	if (EXTI_GetITStatus(key0_EXTILine) != RESET) // key0_EXIT
	{
		// LED2 取反
		printf("进入到按键0中断服务函数里面\n");
		Led0_Change();
		// 清除中断标志位
		EXTI_ClearITPendingBit(key0_EXTILine);
	}
}
// VOLMode中断服务函数
void EXTI9_5_IRQHandler(void)
{
	// // 确保是否产生了 EXTI Line 中断
	// if (EXTI_GetITStatus(VOLMode_EXTILine) != RESET) // key1_EXTILine key1_EXIT
	// {
	// 	// LED2 取反
	// 	printf("进入到 电源模式 中断服务函数里面\n");
	// 	Led1_Change();
	// 	VOLModeChanged1();
	// 	// 清除中断标志位
	// 	EXTI_ClearITPendingBit(VOLMode_EXTILine);
	// }
	// else if (EXTI_GetITStatus(VOLContrl_EXTILine) != RESET) // key0_EXIT
	// {
	// 	// delay_ms(20);											//延时消抖
	// 	// while (EXTI_GetITStatus(VOLContrl_EXTILine) != RESET);	//等待按键松手
	// 	// delay_ms(20);

	// 	// LED2 取反
	// 	printf("进入到 电源启停 中断服务函数里面\n");
	// 	Led0_Change();
	// 	// 清除中断标志位
	// 	EXTI_ClearITPendingBit(VOLContrl_EXTILine);
	// }
}
// //VOLControl中断服务函数
// void EXTI9_5_IRQHandler(void)
// {
//   // 确保是否产生了 EXTI Line 中断
// if (EXTI_GetITStatus(VOLContrl_EXTILine) != RESET)//key0_EXIT
// {
//   // LED2 取反
//   printf("进入到 电源启停 中断服务函数里面\n");
//   Led0_Change();
//   // 清除中断标志位
//   EXTI_ClearITPendingBit(VOLContrl_EXTILine);
// }
// }

// FLOWMode中断服务函数
void EXTI1_IRQHandler(void)
{
	// 确保是否产生了 EXTI Line 中断
	if (EXTI_GetITStatus(emStop_EXTILine) != RESET) // key1_EXTILine key1_EXIT
	{
		// LED2 取反
		printf("进入到 流量模式 中断服务函数里面\n");
		Led1_Change();
		
		// 清除中断标志位
		EXTI_ClearITPendingBit(emStop_EXTILine);
	}
}
// FLOWControl中断服务函数
void EXTI2_IRQHandler(void)
{
	// 确保是否产生了 EXTI Line 中断
	// if (EXTI_GetITStatus(FLOWContrl_EXTILine) != RESET) // key0_EXIT
	// {
	// 	// LED2 取反
	// 	printf("进入到 流量启停 中断服务函数里面\n");
	// 	Led0_Change();
	// 	// 清除中断标志位
	// 	EXTI_ClearITPendingBit(FLOWContrl_EXTILine);
	// }
}

void ADC1_2_IRQHandler(void)
{
	// if (ADC_GetITStatus(ADCx, ADC_IT_EOC) == SET)
	// {
	//   // 读取 ADC 的转换值
	//   ADC_ConvertedValue = ADC_GetConversionValue(ADCx);
	// }
	// ADC_ClearITPendingBit(ADCx, ADC_IT_EOC);
}


#include "cmd.h"
//命令串口中断服务函数
void USART1_IRQHandler(void)
{
	
	if (USART_GetITStatus(CmdUsart, USART_IT_RXNE) != RESET)
    {
        // 接收数据
        uint8_t received_char = USART_ReceiveData(CmdUsart);

		// 防止缓冲区溢出
		if(rxIndex < RX_BUFFER_SIZE - 1) {
			recv_buf[rxIndex++]=received_char;
		} else {
			// 缓冲区满，重置索引
			rxIndex = 0;
			recv_buf[rxIndex++]=received_char;
		}
		
        // 发送接收到的数据
        // USART_SendData(USART2, received_char);

        // 等待发送数据寄存器空
        // while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

        // 清除中断标志位
        USART_ClearITPendingBit(CmdUsart, USART_IT_RXNE);
    }
}
//日志串口中断服务函数
void USART2_IRQHandler(void)
{
	
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        // 接收数据
        uint8_t received_char = USART_ReceiveData(USART2);

		// 防止缓冲区溢出
		if(rxIndex < RX_BUFFER_SIZE - 1) {
			recv_buf[rxIndex++]=received_char;
		} else {
			// 缓冲区满，重置索引
			rxIndex = 0;
			recv_buf[rxIndex++]=received_char;
		}
		
        // 清除中断标志位
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

	// uint8_t received_char;
	// // 接收数据
	// if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	// {

	// 	received_char = USART_ReceiveData(USART1);
	// 	static uint8_t temp_buf[2];//判断包头的缓冲区
	// 	int temp_index = 0;

	// 	// 过滤掉开头的'\r'和'\n'
	// 	if (((received_char == '\r') && (RecvBuf_Index == 0)) || ((received_char == '\n') && (RecvBuf_Index == 0)))
	// 	{
	// 		//重置索引
	// 		RecvBuf_Index = 0;
	// 		temp_index = 0;
	// 		return;
	// 	}

	// 	// 如果接收到换行符，表示字符串结束
	// 	if ((received_char == '\r') && (RecvBuf_Index > 1))
	// 	{
	// 		RecvBuf[RecvBuf_Index] = '\0'; // 在接收缓冲区末尾添加字符串结束符
	// 		RecvBuf_Index = 0;			   // 重置接收缓冲区索引
	// 		bufReady = 1;
	// 		printf("接收到的字符为:%s\n", RecvBuf);
	// 		return;
	// 	}
	// 	else
	// 	{
	// 		// 将接收的数据放入缓冲区
	// 		RecvBuf[RecvBuf_Index++] = received_char; // 将接收到的字符存入接收缓冲区
	// 												  // USART_SendData(USART1,received_char);
	// 		temp_buf[temp_index]=received_char;
	// 		if(temp_index==0) 
	// 		{
	// 			temp_index = 1;
	// 		}else if(temp_index==1){
	// 			temp_index = 0;
	// 			printf("tempbuf:%d\n",temp_buf[0]<<8|temp_buf[1]);
	// 		}
			
			
			

	// 		if (RecvBuf_Index >= BUFSIZE)
	// 		{
	// 			RecvBuf[BUFSIZE - 1] = '\0';
	// 			RecvBuf_Index = 0; // 如果超出，重置索引
	// 			bufReady = 2;
	// 		}
	// 	}
	// }
// }

void TIM3_IRQHandler(void)
{
	static int pulse_count = 0;
	printf("进入到定时器中断!\n");
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		printf("pulse_count = %d,Angle=%d\n", pulse_count,Angle);
		pulse_count++;

		
			if (pulse_count >= Angle)//400
			{
				// 停止TIM3
				TIM_Cmd(TIM3, DISABLE);

				// 重置计数器
				pulse_count = 0;

				
				printf("转动完成，调压器输出值应为:%d\n",Angles);
			}
		/*
		if (Keynum == 4)
		{
			if (pulse_count >= 800)
			{
				// 停止TIM3
				TIM_Cmd(TIM3, DISABLE);

				// 重置计数器
				pulse_count = 0;
			}
		}*/
	}
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
