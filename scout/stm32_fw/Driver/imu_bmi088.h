#pragma once
#include "mahony.h"
typedef struct { float accel[3], gyro[3], temp; } RawImu_t;
extern RawImu_t  g_imu_raw;
extern Mahony_t  g_ahrs;
void IMU_Init  (void);
void IMU_Update(float dt);   /* 500Hz 调用 */
