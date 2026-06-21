#pragma once
/** @file supply.h -- 物资投送舵机控制模块 */
#include <stdint.h>

#define SUPPLY_SLOT_COUNT 4  /* 投送槽位数 */

void Supply_Init(void);          /* 初始化 TIM3 PWM */
void Supply_Deploy(uint8_t slot); /* 打开指定槽位并开始投送 */
void Supply_Close(uint8_t slot);  /* 关闭指定槽位 */
void Supply_Update(void);         /* 状态机更新, 自动关仓计时 */
