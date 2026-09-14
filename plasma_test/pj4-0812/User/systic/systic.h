#ifndef _SYSTIC_H
#define _SYSTIC_H

#include "stm32f10x.h"
#include "core_cm3.h"


extern int timeflag;



void systick_interuppt(uint32_t ms);
void delay_ms(uint32_t ntimes);
#endif
