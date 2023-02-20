#ifndef RGB_LED_H
#define RGB_LED_H

#include "sys.h"

void TIM3_PWM_Init(void);
void SetLedColor(uint8_t (*color)[3], uint16_t len);
void RgbLedTask(void *arg);



#endif


