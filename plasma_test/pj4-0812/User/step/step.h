#ifndef _STEP_H_
#define _STEP_H_

#include "stm32f10x.h"

#define ZHENG 0
#define FAN 1

extern uint8_t StepDir;
extern int Angle;

#define StepStart 3
#define StepStop 4

#define StepUpDir_flag 5
#define StepDownDir_flag 6

extern u8 StepWorkUp_flag;
extern u8 StepWorkDown_flag;
extern u32 Angles;
extern u32 SingleAngles;

void TIM3_PWM_Init(u16 arr, u16 psc);
void DirGpioInit(void);
void SetStepDir(int dir);//调整点击转动方向
void StopStep(u8 dir);//停止计时器
void StartStep(u8 dir);//开启步进电机
void ZeroModify(u16 angle);
void step_1v(void);

void Step_init(void);//电机模块初始化

void step90_zheng(void);
void step90_fan(void);


#endif


