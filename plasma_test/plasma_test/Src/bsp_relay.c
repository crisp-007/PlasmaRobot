#include "bsp_relay.h"

#define HE_RELAY_GPIO_PORT  GPIOC
#define HE_RELAY_GPIO_PIN   GPIO_PIN_3
#define AR_RELAY_GPIO_PORT  GPIOC
#define AR_RELAY_GPIO_PIN   GPIO_PIN_2
#define VOL_MOD_RELAY_GPIO_PORT  GPIOC
#define VOL_MOD_RELAY_GPIO_PIN   GPIO_PIN_1
#define PLASMA_RELAY_GPIO_PORT   GPIOA
#define PLASMA_RELAY_GPIO_PIN    GPIO_PIN_7

#define RELAY_ON_LEVEL      GPIO_PIN_SET
#define RELAY_OFF_LEVEL     GPIO_PIN_RESET

static void RelayWrite(GPIO_TypeDef *port, uint16_t pin, bool on)
{
  HAL_GPIO_WritePin(port, pin, on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
}

static bool RelayRead(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == RELAY_ON_LEVEL;
}

void BoardRelay_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(HE_RELAY_GPIO_PORT, HE_RELAY_GPIO_PIN, RELAY_OFF_LEVEL);
  HAL_GPIO_WritePin(AR_RELAY_GPIO_PORT, AR_RELAY_GPIO_PIN, RELAY_OFF_LEVEL);
  HAL_GPIO_WritePin(VOL_MOD_RELAY_GPIO_PORT, VOL_MOD_RELAY_GPIO_PIN, RELAY_OFF_LEVEL);
  HAL_GPIO_WritePin(PLASMA_RELAY_GPIO_PORT, PLASMA_RELAY_GPIO_PIN, RELAY_OFF_LEVEL);

  GPIO_InitStruct.Pin = HE_RELAY_GPIO_PIN | AR_RELAY_GPIO_PIN | VOL_MOD_RELAY_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PLASMA_RELAY_GPIO_PIN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void BoardRelay_Set(BoardRelay_Id relay, bool on)
{
  switch (relay)
  {
  case he_relay:
    RelayWrite(HE_RELAY_GPIO_PORT, HE_RELAY_GPIO_PIN, on);
    break;

  case ar_relay:
    RelayWrite(AR_RELAY_GPIO_PORT, AR_RELAY_GPIO_PIN, on);
    break;

  case vol_mod_relay:
    RelayWrite(VOL_MOD_RELAY_GPIO_PORT, VOL_MOD_RELAY_GPIO_PIN, on);
    break;

  case plasma_relay:
    RelayWrite(PLASMA_RELAY_GPIO_PORT, PLASMA_RELAY_GPIO_PIN, on);
    break;

  default:
    break;
  }
}

bool BoardRelay_Get(BoardRelay_Id relay)
{
  switch (relay)
  {
  case he_relay:
    return RelayRead(HE_RELAY_GPIO_PORT, HE_RELAY_GPIO_PIN);

  case ar_relay:
    return RelayRead(AR_RELAY_GPIO_PORT, AR_RELAY_GPIO_PIN);

  case vol_mod_relay:
    return RelayRead(VOL_MOD_RELAY_GPIO_PORT, VOL_MOD_RELAY_GPIO_PIN);

  case plasma_relay:
    return RelayRead(PLASMA_RELAY_GPIO_PORT, PLASMA_RELAY_GPIO_PIN);

  default:
    return false;
  }
}
