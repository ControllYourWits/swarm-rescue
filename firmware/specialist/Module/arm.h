#pragma once
/** @file arm.h -- 4-DOF 机械臂舵机控制模块 */
#include <stdint.h>

void Arm_Init(void);                     /* 初始化 TIM3 PWM, 归位 */
void Arm_SetJoint(uint8_t joint, float angle_deg); /* 设置目标关节角度 (度) */
void Arm_SetGripper(uint8_t state);       /* 设置夹爪状态: 0=松开, 1=夹紧 */
void Arm_Update(void);                    /* 插值更新, 以 100Hz 调用 */
