#ifndef __BSP_RELAY_H__
#define __BSP_RELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

typedef enum
{
  he_relay = 0,
  ar_relay,
  vol_mod_relay,
  plasma_relay
} BoardRelay_Id;

void BoardRelay_Init(void);
void BoardRelay_Set(BoardRelay_Id relay, bool on);
bool BoardRelay_Get(BoardRelay_Id relay);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_RELAY_H__ */
