#include "led.h"
#include "stm32f10x.h"
#include "usart1.h"

void Led0_Init(void)
{
    //开启对应时钟
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    RCC_APB2PeriphClockCmd(LED0_CLK,ENABLE);
    GPIO_InitTypeDef LedGpio_Iint;
    /* 选择要控制的 GPIO 引脚 */
    LedGpio_Iint.GPIO_Pin = LED0_PIN;
    /* 设置引脚模式为通用推挽输出 */
    LedGpio_Iint.GPIO_Mode = GPIO_Mode_Out_PP;
    /* 设置引脚速率为 50MHz */
    LedGpio_Iint.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED0_PORT,&LedGpio_Iint);
    Led0_OFF();
}

void Led0_ON(void)
{
    GPIO_ResetBits(LED0_PORT,LED0_PIN);
}
void Led0_OFF(void)
{
    GPIO_SetBits(LED0_PORT,LED0_PIN);
}

void Led0_Change(void)
{
    int ref=GPIO_ReadOutputDataBit(LED0_PORT,LED0_PIN);
    printf("led-D0当前状态为:%d\n",ref);
    if(ref==1)
    {
        Led0_ON();
    }else
    {
        Led0_OFF();
    }
}


void Led1_Init(void)
{
    //开启对应时钟
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    RCC_APB2PeriphClockCmd(LED1_CLK,ENABLE);
    GPIO_InitTypeDef LedGpio_Iint;
    /* 选择要控制的 GPIO 引脚 */
    LedGpio_Iint.GPIO_Pin = LED1_PIN;
    /* 设置引脚模式为通用推挽输出 */
    LedGpio_Iint.GPIO_Mode = GPIO_Mode_Out_PP;
    /* 设置引脚速率为 50MHz */
    LedGpio_Iint.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED1_PORT,&LedGpio_Iint);
    Led1_OFF();
}

void Led1_ON(void)
{
    GPIO_ResetBits(LED1_PORT,LED1_PIN);
}
void Led1_OFF(void)
{
    GPIO_SetBits(LED1_PORT,LED1_PIN);
}

void Led1_Change(void)
{
    int ref=GPIO_ReadOutputDataBit(LED1_PORT,LED1_PIN);
    printf("led-D1当前状态为:%d\n",ref);
    if(ref==1)
    {
        Led1_ON();
    }else
    {
        Led1_OFF();
    }
}

//=== 三色LED控制函数实现 ===

// 全局变量：记录当前设置的颜色
static uint8_t current_rgb_color = RGB_COLOR_NONE;

// 三色LED初始化
void Rgb_Init(void)
{
    // 开启GPIOC时钟
    RCC_APB2PeriphClockCmd(RGB_LED_CLK, ENABLE);
    
    GPIO_InitTypeDef RgbLedGpio_Init;
    // 配置红、绿、黄三个LED引脚
    RgbLedGpio_Init.GPIO_Pin = RGB_LED_RED_PIN | RGB_LED_GREEN_PIN | RGB_LED_YELLOW_PIN;
    RgbLedGpio_Init.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    RgbLedGpio_Init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RGB_LED_PORT, &RgbLedGpio_Init);
    
    // 初始化时关闭所有LED
    current_rgb_color = RGB_COLOR_NONE;
    RgbOff();
    printf("三色LED初始化完成\n");
}

// 设置LED颜色（不立即点亮）
void RgbSetColor(uint8_t color)
{
    current_rgb_color = color;
    printf("三色LED颜色设置为: ");
    if(color & RGB_COLOR_RED) printf("红色 ");
    if(color & RGB_COLOR_GREEN) printf("绿色 ");
    if(color & RGB_COLOR_YELLOW) printf("黄色 ");
    if(color == RGB_COLOR_NONE) printf("无");
    printf("\n");
}

// 根据设置的颜色点亮LED
void RgbOn(void)
{
    // 先关闭所有LED
    GPIO_ResetBits(RGB_LED_PORT, RGB_LED_RED_PIN | RGB_LED_GREEN_PIN | RGB_LED_YELLOW_PIN);
    
    // 根据设置的颜色点亮对应的LED
    if(current_rgb_color & RGB_COLOR_RED)
    {
        GPIO_SetBits(RGB_LED_PORT, RGB_LED_RED_PIN);
    }
    if(current_rgb_color & RGB_COLOR_GREEN)
    {
        GPIO_SetBits(RGB_LED_PORT, RGB_LED_GREEN_PIN);
    }
    if(current_rgb_color & RGB_COLOR_YELLOW)
    {
        GPIO_SetBits(RGB_LED_PORT, RGB_LED_YELLOW_PIN);
    }
    
    printf("三色LED点亮\n");
}

// 关闭所有LED
void RgbOff(void)
{
    GPIO_ResetBits(RGB_LED_PORT, RGB_LED_RED_PIN | RGB_LED_GREEN_PIN | RGB_LED_YELLOW_PIN);
    printf("三色LED关闭\n");
}


