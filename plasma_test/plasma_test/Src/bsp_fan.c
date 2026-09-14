#include "bsp_fan.h"

#define FAN_PWM_PERIOD       999U
#define FAN_PWM_FULL_DUTY    1000U

static uint16_t fan_duty[3] = {0U};

static uint32_t FanDutyToPulse(uint16_t duty_permille)
{
  if (duty_permille > FAN_PWM_FULL_DUTY)
  {
    duty_permille = FAN_PWM_FULL_DUTY;
  }

  return ((FAN_PWM_PERIOD + 1U) * duty_permille) / FAN_PWM_FULL_DUTY;
}

static void FanSetPulse(BoardFan_Id fan, uint32_t pulse)
{
  switch (fan)
  {
  case ctr_ass_fan:
    TIM3->CCR2 = pulse;
    break;

  case device_fan:
    TIM4->CCR1 = pulse;
    break;

  case ctr_main_fan:
    TIM4->CCR2 = pulse;
    break;

  default:
    break;
  }
}

void BoardFan_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  TIM3->CR1 = 0U;
  TIM3->PSC = 84U - 1U;
  TIM3->ARR = FAN_PWM_PERIOD;
  TIM3->CCR2 = 0U;
  TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
  TIM3->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM3->CCER &= ~TIM_CCER_CC2P;
  TIM3->CCER |= TIM_CCER_CC2E;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  TIM4->CR1 = 0U;
  TIM4->PSC = 84U - 1U;
  TIM4->ARR = FAN_PWM_PERIOD;
  TIM4->CCR1 = 0U;
  TIM4->CCR2 = 0U;
  TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S | TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
  TIM4->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                 (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
  TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void BoardFan_Set(BoardFan_Id fan, bool on)
{
  BoardFan_SetDuty(fan, on ? FAN_PWM_FULL_DUTY : 0U);
}

bool BoardFan_Get(BoardFan_Id fan)
{
  return BoardFan_GetDuty(fan) > 0U;
}

void BoardFan_SetDuty(BoardFan_Id fan, uint16_t duty_permille)
{
  if (fan > ctr_main_fan)
  {
    return;
  }

  if (duty_permille > FAN_PWM_FULL_DUTY)
  {
    duty_permille = FAN_PWM_FULL_DUTY;
  }

  fan_duty[fan] = duty_permille;
  FanSetPulse(fan, FanDutyToPulse(duty_permille));
}

uint16_t BoardFan_GetDuty(BoardFan_Id fan)
{
  if (fan > ctr_main_fan)
  {
    return 0U;
  }

  return fan_duty[fan];
}
