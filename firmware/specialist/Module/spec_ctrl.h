#pragma once
#include <stdint.h>

void Spec_Init(void);
void Spec_SetChassis(float vx, float vy, float wz, uint8_t mode);
void Spec_ChassisUpdate(void);
void Spec_GetOdom(float *x, float *y, float *yaw);
void Spec_GetVelocity(float *vx, float *vy, float *wz);
uint8_t Spec_GetMode(void);
