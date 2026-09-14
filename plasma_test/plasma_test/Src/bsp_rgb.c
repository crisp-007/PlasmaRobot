#include "bsp_rgb.h"

#define RGB_RED_GPIO_PORT    GPIOE
#define RGB_RED_GPIO_PIN     GPIO_PIN_8
#define RGB_GREEN_GPIO_PORT  GPIOE
#define RGB_GREEN_GPIO_PIN   GPIO_PIN_9
#define RGB_YELLOW_GPIO_PORT GPIOE
#define RGB_YELLOW_GPIO_PIN  GPIO_PIN_10

#define RGB_ON_LEVEL         GPIO_PIN_SET
#define RGB_OFF_LEVEL        GPIO_PIN_RESET

static void RgbWrite(GPIO_TypeDef *port, uint16_t pin, bool on)
{
  HAL_GPIO_WritePin(port, pin, on ? RGB_ON_LEVEL : RGB_OFF_LEVEL);
}

static bool RgbRead(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == RGB_ON_LEVEL;
}

void BoardRgb_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, RGB_RED_GPIO_PIN | RGB_GREEN_GPIO_PIN | RGB_YELLOW_GPIO_PIN, RGB_OFF_LEVEL);

  GPIO_InitStruct.Pin = RGB_RED_GPIO_PIN | RGB_GREEN_GPIO_PIN | RGB_YELLOW_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void BoardRgb_Set(BoardRgb_Id color, bool on)
{
  switch (color)
  {
  case rgb_red:
    RgbWrite(RGB_RED_GPIO_PORT, RGB_RED_GPIO_PIN, on);
    break;

  case rgb_green:
    RgbWrite(RGB_GREEN_GPIO_PORT, RGB_GREEN_GPIO_PIN, on);
    break;

  case rgb_yellow:
    RgbWrite(RGB_YELLOW_GPIO_PORT, RGB_YELLOW_GPIO_PIN, on);
    break;

  default:
    break;
  }
}

bool BoardRgb_Get(BoardRgb_Id color)
{
  switch (color)
  {
  case rgb_red:
    return RgbRead(RGB_RED_GPIO_PORT, RGB_RED_GPIO_PIN);

  case rgb_green:
    return RgbRead(RGB_GREEN_GPIO_PORT, RGB_GREEN_GPIO_PIN);

  case rgb_yellow:
    return RgbRead(RGB_YELLOW_GPIO_PORT, RGB_YELLOW_GPIO_PIN);

  default:
    return false;
  }
}

void BoardRgb_AllOff(void)
{
  BoardRgb_Set(rgb_red, false);
  BoardRgb_Set(rgb_green, false);
  BoardRgb_Set(rgb_yellow, false);
}
