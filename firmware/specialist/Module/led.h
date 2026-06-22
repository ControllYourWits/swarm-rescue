#pragma once
/** @file led.h -- RGB LED 阵列控制模块 */
#include <stdint.h>

#define LED_MODE_OFF      0  /* 关闭 */
#define LED_MODE_CONSTANT 1  /* 常亮 */
#define LED_MODE_STROBE   2  /* 5Hz 频闪 */
#define LED_MODE_SOS      3  /* SOS 国际求救信号 (...---...) */

void LED_Init(void);
void LED_SetMode(uint8_t mode, uint8_t brightness,
                 uint8_t r, uint8_t g, uint8_t b);
void LED_Update(void);
