#ifndef __BSP_FAN_H__
#define __BSP_FAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

typedef enum
{
  ctr_ass_fan = 0,
  device_fan,
  ctr_main_fan
} BoardFan_Id;

void BoardFan_Init(void);
void BoardFan_Set(BoardFan_Id fan, bool on);
bool BoardFan_Get(BoardFan_Id fan);
void BoardFan_SetDuty(BoardFan_Id fan, uint16_t duty_permille);
uint16_t BoardFan_GetDuty(BoardFan_Id fan);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FAN_H__ */
