#include "kinematics.h"
#include <math.h>
#define WHEEL_R   0.0765f
#define LXY       0.2025f
#define RPM2RADS  (2.0f * 3.14159265f / 60.0f)

void Kin_Inverse(float vx, float vy, float wz, float rpm[4]) {
    float k = 1.0f / WHEEL_R / RPM2RADS;
    rpm[0] = ( vx - vy - LXY * wz) * k;
    rpm[1] = ( vx + vy + LXY * wz) * k;
    rpm[2] = ( vx - vy + LXY * wz) * k;
    rpm[3] = ( vx + vy - LXY * wz) * k;
}

void Kin_Forward(const float rpm[4], float *vx, float *vy, float *wz) {
    float k = WHEEL_R * RPM2RADS;
    *vx = ( rpm[0] + rpm[1] + rpm[2] + rpm[3]) * k * 0.25f;
    *vy = (-rpm[0] + rpm[1] - rpm[2] + rpm[3]) * k * 0.25f;
    *wz = (-rpm[0] + rpm[1] + rpm[2] - rpm[3]) * k / (4.0f * LXY);
}
