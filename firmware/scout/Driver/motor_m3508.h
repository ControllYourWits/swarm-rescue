#pragma once
#include <stdint.h>
#define MOTOR_NUM 4
typedef struct {
    int16_t  angle;
    int16_t  speed_rpm;
    int16_t  torque;
    uint8_t  temp;
    uint8_t  online;
} MotorFB_t;
extern MotorFB_t g_motor[MOTOR_NUM];
void Motor_Init(void);
void Motor_Send(int16_t i1, int16_t i2, int16_t i3, int16_t i4);
void Motor_Parse(uint32_t id, const uint8_t *d);
void Motor_OfflineCheck(void);   /* 在 1ms 定时器中调用 */
