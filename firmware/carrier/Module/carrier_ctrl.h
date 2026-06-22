#pragma once
#include <stdint.h>

#define SUPPLY_CLOSE 0U
#define SUPPLY_OPEN  1U
#define SUPPLY_THROW 2U

void Carrier_Init(void);
void Carrier_SetVel(float vx, float vy, float wz, uint8_t mode);
void Carrier_Update(void);
void Carrier_SupplyAction(uint8_t action, uint8_t slot);
void Carrier_GetOdom(float *x, float *y, float *yaw);
void Carrier_GetVelocity(float *vx, float *vy, float *wz);
uint8_t Carrier_GetMode(void);
float Carrier_GetBattery(void);
float Carrier_GetBatteryVoltage(void);
