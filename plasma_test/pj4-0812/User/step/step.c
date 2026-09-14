#include "step.h"
#include "usart1.h"

// 声明Direction和Keynum为全局变量
uint8_t StepDir = ZHENG;
int Angle=0;
u32 Angles=0;

u8 StepWorkUp_flag=StepStop;
u8 StepWorkDown_flag=StepStop;
u32 SingleAngles=0;


void TIM3_PWM_Init(u16 arr, u16 psc) /// pwm
{
    GPIO_InitTypeDef GPIO_InitStrue;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // 引脚初始化
    GPIO_InitStrue.GPIO_Pin = GPIO_Pin_0; // PB0->CH3  PB1->CH4 TIM3
    GPIO_InitStrue.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStrue.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStrue);

    // 基本定时器配置
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc -1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 定时器比较输出初始化
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    // TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 启用TIM3中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    
    TIM_Cmd(TIM3, DISABLE); // 禁用定时器，等待被调用
}


void DirGpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 调整点击转动方向
void SetStepDir(int dir)
{
    StepDir=dir;
    if (StepDir == ZHENG)
    {
        // 正转
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    }
    else if (StepDir == FAN)
    {
        // 反转
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
    }
}
// 步进电机初始化
void Step_init(void)
{

    // 引脚初始化
    //3200-16细分
    // TIM3_PWM_Init(313, 72);600
    TIM3_PWM_Init(312, 72);

    DirGpioInit();

    // 功能初始化
    SetStepDir(StepDir);
}

void ZeroModify(u16 angle)
{
    printf("最后的修正-%d\n",angle);
    //1、设置转动方向
    SetStepDir(FAN);
    //2、设置转动角度和速度
    TIM_Cmd(TIM3, DISABLE);
    TIM_SetCompare3(TIM3, 150);
    Angle=angle;//半圈
    // TIM3_PWM_Init(1000, 72);//速度
    TIM_Cmd(TIM3, ENABLE);
}


void step_1v(void)
{
    if(StepWorkUp_flag==StepStop) return;

    
    // int s1=30;
    while(Angles<17)
    {
        printf("进入到正转程序中\n");
        //1、设置转动方向
        SetStepDir(ZHENG);
        //2、设置转动角度和速度
        TIM_Cmd(TIM3, DISABLE);
        TIM_SetCompare3(TIM3, 150);
        Angle=1;//半圈
        // if()
        Angles+=Angle;//累计正转多少
        // TIM3_PWM_Init(1000, 72);//速度
        TIM_Cmd(TIM3, ENABLE);
    }
    
    
    // return 0;
}

void step90_zheng(void)
{
    if(StepWorkUp_flag==StepStop)
    {
        printf("进入正转但标志为停止\n");
        return;
    }
    

    printf("进入到正转程序中\n");
    //1、设置转动方向
    SetStepDir(ZHENG);
    //2、设置转动角度和速度
    TIM_Cmd(TIM3, DISABLE);
    TIM_SetCompare3(TIM3, 150);
    Angle=20;//半圈
    // if()
    Angles+=Angle;//累计正转多少
    // TIM3_PWM_Init(1000, 72);//速度
    TIM_Cmd(TIM3, ENABLE);
    
    // return 0;
}

void step90_fan(void)
{
    if(StepWorkDown_flag==StepStop) 
    {
        printf("进入反转但标志为停止\n");
        return;
    }

    printf("进入到反转程序中\n");
    //1、设置转动方向
    SetStepDir(FAN);
    //2、设置转动角度和速度
    TIM_Cmd(TIM3, DISABLE);
    TIM_SetCompare3(TIM3, 150);
    Angle=5;//半圈
    Angles-=Angle;//累计反转多少
    // TIM3_PWM_Init(1000, 72);//速度
    TIM_Cmd(TIM3, ENABLE);
    
    // return 0;
}

void StopStep(u8 dir)
{
    //1、停止计时器
    // TIM_Cmd(TIM3, DISABLE);
    //2、停止转动的方向
    if(dir ==StepUpDir_flag)
    {
        StepWorkUp_flag=StepStop;
    }else if(dir ==StepDownDir_flag)
    {
        StepWorkDown_flag=StepStop;
    }
    Angle=0;//不发送脉冲
    printf("StepStop!\n");
}

void StartStep(u8 dir)
{
    // TIM_Cmd(TIM3, ENABLE);//开启计时器

    if(dir ==StepUpDir_flag)
    {
        StepWorkUp_flag=StepStart;
    }else if(dir ==StepDownDir_flag)
    {
        StepWorkDown_flag=StepStart;
    }
    printf("StepStart!\n");
}

