#ifndef _LED_H_
#define _LED_H_

#include "stm32f10x.h"

//LED0端口及引脚
#define LED0_PORT GPIOB
#define LED0_PIN GPIO_Pin_5
#define LED0_CLK RCC_APB2Periph_GPIOB
//LED1端口及引脚
#define LED1_PORT GPIOE
#define LED1_PIN GPIO_Pin_5
#define LED1_CLK RCC_APB2Periph_GPIOE

//led0初始化
void Led0_Init(void);
void Led0_ON(void);
void Led0_OFF(void);
void Led0_Change(void);

//led1初始化
void Led1_Init(void);
void Led1_ON(void);
void Led1_OFF(void);
void Led1_Change(void);

//三色LED端口及引脚定义
#define RGB_LED_PORT GPIOC
#define RGB_LED_RED_PIN GPIO_Pin_6    // 红色LED引脚
#define RGB_LED_GREEN_PIN GPIO_Pin_7  // 绿色LED引脚
#define RGB_LED_YELLOW_PIN GPIO_Pin_8 // 黄色LED引脚
#define RGB_LED_CLK RCC_APB2Periph_GPIOC

//三色LED颜色定义（可组合使用）
#define RGB_COLOR_NONE   0x00   // 无颜色
#define RGB_COLOR_RED    0x01   // 红色
#define RGB_COLOR_GREEN  0x02   // 绿色
#define RGB_COLOR_YELLOW 0x04   // 黄色

//三色LED初始化和控制函数
void Rgb_Init(void);
void RgbSetColor(uint8_t color);  // 设置要显示的颜色（可组合）
void RgbOn(void);             // 根据设置的颜色点亮LED
void RgbOff(void);            // 关闭所有LED

#endif
