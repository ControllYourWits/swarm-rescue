#pragma once
/* 麦克纳姆轮运动学 - X型安装
 * 轮序(俯视): M1左前 M2右前 M3右后 M4左后 */
void Kin_Inverse(float vx, float vy, float wz, float rpm[4]);
void Kin_Forward(const float rpm[4], float *vx, float *vy, float *wz);
