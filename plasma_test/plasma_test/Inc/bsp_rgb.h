#ifndef __BSP_RGB_H__
#define __BSP_RGB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

typedef enum
{
  rgb_red = 0,
  rgb_green,
  rgb_blue,
  rgb_yellow = rgb_blue
} BoardRgb_Id;

void BoardRgb_Init(void);
void BoardRgb_Set(BoardRgb_Id color, bool on);
bool BoardRgb_Get(BoardRgb_Id color);
void BoardRgb_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_RGB_H__ */
