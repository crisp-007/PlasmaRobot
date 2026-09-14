#include "bsp_gas_valve.h"

#define HE_VALVE_GPIO_PORT       GPIOD
#define HE_VALVE_GPIO_PIN        GPIO_PIN_12
#define AR_VALVE_GPIO_PORT       GPIOD
#define AR_VALVE_GPIO_PIN        GPIO_PIN_13
#define SPARE_VALVE_3_GPIO_PORT  GPIOD
#define SPARE_VALVE_3_GPIO_PIN   GPIO_PIN_14
#define SPARE_VALVE_4_GPIO_PORT  GPIOD
#define SPARE_VALVE_4_GPIO_PIN   GPIO_PIN_15

#define VALVE_ON_LEVEL           GPIO_PIN_SET
#define VALVE_OFF_LEVEL          GPIO_PIN_RESET

static void ValveWrite(GPIO_TypeDef *port, uint16_t pin, bool on)
{
  HAL_GPIO_WritePin(port, pin, on ? VALVE_ON_LEVEL : VALVE_OFF_LEVEL);
}

static bool ValveRead(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == VALVE_ON_LEVEL;
}

void BoardGasValve_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD,
                    HE_VALVE_GPIO_PIN | AR_VALVE_GPIO_PIN |
                    SPARE_VALVE_3_GPIO_PIN | SPARE_VALVE_4_GPIO_PIN,
                    VALVE_OFF_LEVEL);

  GPIO_InitStruct.Pin = HE_VALVE_GPIO_PIN | AR_VALVE_GPIO_PIN |
                        SPARE_VALVE_3_GPIO_PIN | SPARE_VALVE_4_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void BoardGasValve_Set(BoardGasValve_Id valve, bool on)
{
  switch (valve)
  {
  case he_valve:
    ValveWrite(HE_VALVE_GPIO_PORT, HE_VALVE_GPIO_PIN, on);
    break;

  case ar_valve:
    ValveWrite(AR_VALVE_GPIO_PORT, AR_VALVE_GPIO_PIN, on);
    break;

  case spare_valve_3:
    ValveWrite(SPARE_VALVE_3_GPIO_PORT, SPARE_VALVE_3_GPIO_PIN, on);
    break;

  case spare_valve_4:
    ValveWrite(SPARE_VALVE_4_GPIO_PORT, SPARE_VALVE_4_GPIO_PIN, on);
    break;

  default:
    break;
  }
}

bool BoardGasValve_Get(BoardGasValve_Id valve)
{
  switch (valve)
  {
  case he_valve:
    return ValveRead(HE_VALVE_GPIO_PORT, HE_VALVE_GPIO_PIN);

  case ar_valve:
    return ValveRead(AR_VALVE_GPIO_PORT, AR_VALVE_GPIO_PIN);

  case spare_valve_3:
    return ValveRead(SPARE_VALVE_3_GPIO_PORT, SPARE_VALVE_3_GPIO_PIN);

  case spare_valve_4:
    return ValveRead(SPARE_VALVE_4_GPIO_PORT, SPARE_VALVE_4_GPIO_PIN);

  default:
    return false;
  }
}
