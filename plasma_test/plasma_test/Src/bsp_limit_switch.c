#include "bsp_limit_switch.h"

#define INITIAL_LIMIT_GPIO_PORT  GPIOE
#define INITIAL_LIMIT_GPIO_PIN   GPIO_PIN_11
#define MAX_LIMIT_GPIO_PORT      GPIOE
#define MAX_LIMIT_GPIO_PIN       GPIO_PIN_12
#define EMERGENCY_STOP_GPIO_PORT GPIOE
#define EMERGENCY_STOP_GPIO_PIN  GPIO_PIN_13

#define LIMIT_PRESSED_LEVEL      GPIO_PIN_RESET
#define EMERGENCY_STOP_LEVEL     GPIO_PIN_SET

static bool LimitRead(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == LIMIT_PRESSED_LEVEL;
}

void BoardLimitSwitch_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitStruct.Pin = INITIAL_LIMIT_GPIO_PIN | MAX_LIMIT_GPIO_PIN | EMERGENCY_STOP_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

bool BoardLimitSwitch_IsPressed(BoardLimitSwitch_Id limit_switch)
{
  switch (limit_switch)
  {
  case initial_limit_switch:
    return LimitRead(INITIAL_LIMIT_GPIO_PORT, INITIAL_LIMIT_GPIO_PIN);

  case max_limit_switch:
    return LimitRead(MAX_LIMIT_GPIO_PORT, MAX_LIMIT_GPIO_PIN);

  case emergency_stop_switch:
    return HAL_GPIO_ReadPin(EMERGENCY_STOP_GPIO_PORT, EMERGENCY_STOP_GPIO_PIN) == EMERGENCY_STOP_LEVEL;

  default:
    return false;
  }
}
