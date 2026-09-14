#ifndef __BSP_LIMIT_SWITCH_H__
#define __BSP_LIMIT_SWITCH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

typedef enum
{
  initial_limit_switch = 0,
  max_limit_switch,
  emergency_stop_switch
} BoardLimitSwitch_Id;

void BoardLimitSwitch_Init(void);
bool BoardLimitSwitch_IsPressed(BoardLimitSwitch_Id limit_switch);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LIMIT_SWITCH_H__ */
