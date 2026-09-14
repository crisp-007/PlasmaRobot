#ifndef __BSP_GAS_VALVE_H__
#define __BSP_GAS_VALVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

typedef enum
{
  he_valve = 0,
  ar_valve,
  spare_valve_3,
  spare_valve_4
} BoardGasValve_Id;

void BoardGasValve_Init(void);
void BoardGasValve_Set(BoardGasValve_Id valve, bool on);
bool BoardGasValve_Get(BoardGasValve_Id valve);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GAS_VALVE_H__ */
