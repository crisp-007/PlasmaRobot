#include "systic.h"
int timeflag=0;

void systick_interuppt(uint32_t ms)
{
    /*操作寄存器的方式*/
    int i;
    SysTick_Config(72000);

    for(i=0;i<ms;i++)
    {
        // 当计数器的值减小到 0 的时候，CRTL 寄存器的位 16 会置 1
        // 当置 1 时，读取该位会清 0
        while(!((SysTick->CTRL)&(1<<16)));
        // 关闭 SysTick 定时器
        SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
    }
}

void delay_ms(uint32_t ntimes)
{
    // NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
    SysTick_Config(72000);
    timeflag=ntimes;
    while(timeflag!=0);
}
