#ifndef __BSP_BUZZER_H__
#define __BSP_BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

typedef enum
{
  buzzer_1 = 0,
  buzzer_2
} BoardBuzzer_Id;

typedef enum
{
  BUZZER_MODE_OFF = 0,
  BUZZER_MODE_SINGLE,
  BUZZER_MODE_LONG,
  BUZZER_MODE_REPEAT_SINGLE
} BoardBuzzer_Mode;

void BoardBuzzer_Init(void);
void BoardBuzzer_Set(BoardBuzzer_Id buzzer, bool on);
void BoardBuzzer_Start(BoardBuzzer_Id buzzer, BoardBuzzer_Mode mode);
void BoardBuzzer_StartRepeat(BoardBuzzer_Id buzzer, uint8_t count);
void BoardBuzzer_Process(void);
bool BoardBuzzer_Get(BoardBuzzer_Id buzzer);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BUZZER_H__ */
