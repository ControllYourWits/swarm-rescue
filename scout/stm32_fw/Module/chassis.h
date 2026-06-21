#pragma once
#include <stdint.h>
void  Chassis_Init       (void);
void  Chassis_SetTarget  (float vx, float vy, float wz, uint8_t mode);
void  Chassis_Update     (void);          /* 1kHz 调用 */
void  Chassis_GetActual  (float *vx, float *vy, float *wz);
void  Chassis_GetOdom    (float *x, float *y, float *yaw);
void  Chassis_SetPwrLimit(float watts);
