#include "bsp_buzzer.h"

#define BUZZER_1_GPIO_PORT  GPIOC
#define BUZZER_1_GPIO_PIN   GPIO_PIN_4
#define BUZZER_2_GPIO_PORT  GPIOC
#define BUZZER_2_GPIO_PIN   GPIO_PIN_5

#define BUZZER_ON_LEVEL     GPIO_PIN_SET
#define BUZZER_OFF_LEVEL    GPIO_PIN_RESET
#define BUZZER_SINGLE_ON_MS 1500U
#define BUZZER_REPEAT_OFF_MS 500U

typedef struct
{
  BoardBuzzer_Mode mode;
  uint32_t start_tick;
  uint8_t remaining_count;
  bool output_on;
} BuzzerState_t;

static BuzzerState_t buzzer_state[2] = {0};

static void BuzzerWrite(GPIO_TypeDef *port, uint16_t pin, bool on)
{
  HAL_GPIO_WritePin(port, pin, on ? BUZZER_ON_LEVEL : BUZZER_OFF_LEVEL);
}

static bool BuzzerRead(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == BUZZER_ON_LEVEL;
}

static void BoardBuzzer_Drive(BoardBuzzer_Id buzzer, bool on)
{
  switch (buzzer)
  {
  case buzzer_1:
    BuzzerWrite(BUZZER_1_GPIO_PORT, BUZZER_1_GPIO_PIN, on);
    break;

  case buzzer_2:
    BuzzerWrite(BUZZER_2_GPIO_PORT, BUZZER_2_GPIO_PIN, on);
    break;

  default:
    break;
  }
}

void BoardBuzzer_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, BUZZER_1_GPIO_PIN | BUZZER_2_GPIO_PIN, BUZZER_OFF_LEVEL);

  GPIO_InitStruct.Pin = BUZZER_1_GPIO_PIN | BUZZER_2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  buzzer_state[buzzer_1].mode = BUZZER_MODE_OFF;
  buzzer_state[buzzer_2].mode = BUZZER_MODE_OFF;
}

void BoardBuzzer_Set(BoardBuzzer_Id buzzer, bool on)
{
  if (buzzer <= buzzer_2)
  {
    buzzer_state[buzzer].mode = on ? BUZZER_MODE_LONG : BUZZER_MODE_OFF;
    buzzer_state[buzzer].start_tick = HAL_GetTick();
    buzzer_state[buzzer].remaining_count = 0U;
    buzzer_state[buzzer].output_on = on;
  }

  BoardBuzzer_Drive(buzzer, on);
}

void BoardBuzzer_Start(BoardBuzzer_Id buzzer, BoardBuzzer_Mode mode)
{
  if (buzzer > buzzer_2)
  {
    return;
  }

  buzzer_state[buzzer].mode = mode;
  buzzer_state[buzzer].start_tick = HAL_GetTick();
  buzzer_state[buzzer].remaining_count = 0U;

  switch (mode)
  {
  case BUZZER_MODE_SINGLE:
  case BUZZER_MODE_LONG:
    buzzer_state[buzzer].output_on = true;
    BoardBuzzer_Drive(buzzer, true);
    break;

  case BUZZER_MODE_OFF:
  default:
    buzzer_state[buzzer].mode = BUZZER_MODE_OFF;
    buzzer_state[buzzer].output_on = false;
    BoardBuzzer_Drive(buzzer, false);
    break;
  }
}

void BoardBuzzer_StartRepeat(BoardBuzzer_Id buzzer, uint8_t count)
{
  if (buzzer > buzzer_2)
  {
    return;
  }

  if (count == 0U)
  {
    BoardBuzzer_Start(buzzer, BUZZER_MODE_OFF);
    return;
  }

  buzzer_state[buzzer].mode = BUZZER_MODE_REPEAT_SINGLE;
  buzzer_state[buzzer].start_tick = HAL_GetTick();
  buzzer_state[buzzer].remaining_count = count;
  buzzer_state[buzzer].output_on = true;
  BoardBuzzer_Drive(buzzer, true);
}

void BoardBuzzer_Process(void)
{
  uint32_t now = HAL_GetTick();

  for (BoardBuzzer_Id buzzer = buzzer_1; buzzer <= buzzer_2; buzzer++)
  {
    if ((buzzer_state[buzzer].mode == BUZZER_MODE_SINGLE) &&
        ((now - buzzer_state[buzzer].start_tick) >= BUZZER_SINGLE_ON_MS))
    {
      BoardBuzzer_Set(buzzer, false);
    }
    else if (buzzer_state[buzzer].mode == BUZZER_MODE_REPEAT_SINGLE)
    {
      if (buzzer_state[buzzer].output_on &&
          ((now - buzzer_state[buzzer].start_tick) >= BUZZER_SINGLE_ON_MS))
      {
        buzzer_state[buzzer].output_on = false;
        buzzer_state[buzzer].start_tick = now;
        if (buzzer_state[buzzer].remaining_count > 0U)
        {
          buzzer_state[buzzer].remaining_count--;
        }
        BoardBuzzer_Drive(buzzer, false);

        if (buzzer_state[buzzer].remaining_count == 0U)
        {
          buzzer_state[buzzer].mode = BUZZER_MODE_OFF;
        }
      }
      else if (!buzzer_state[buzzer].output_on &&
               (buzzer_state[buzzer].remaining_count > 0U) &&
               ((now - buzzer_state[buzzer].start_tick) >= BUZZER_REPEAT_OFF_MS))
      {
        buzzer_state[buzzer].output_on = true;
        buzzer_state[buzzer].start_tick = now;
        BoardBuzzer_Drive(buzzer, true);
      }
    }
  }
}

bool BoardBuzzer_Get(BoardBuzzer_Id buzzer)
{
  switch (buzzer)
  {
  case buzzer_1:
    return BuzzerRead(BUZZER_1_GPIO_PORT, BUZZER_1_GPIO_PIN);

  case buzzer_2:
    return BuzzerRead(BUZZER_2_GPIO_PORT, BUZZER_2_GPIO_PIN);

  default:
    return false;
  }
}
