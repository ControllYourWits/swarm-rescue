/*
 * spec_ctrl.c - Specialist chassis state wrapper plus arm and LED modules.
 *
 * The Specialist shares the same high-level chassis command contract as Scout
 * and Carrier. Motor-specific output can be connected behind Spec_ChassisUpdate
 * without changing the communication layer.
 */
#include "spec_ctrl.h"
#include "arm.h"
#include "led.h"
#include "swarm_protocol.h"
#include <math.h>

#define CTRL_DT 0.001f
#define MAX_VX  0.60f
#define MAX_VY  0.40f
#define MAX_WZ  1.50f
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float s_odom_x = 0.0f;
static float s_odom_y = 0.0f;
static float s_odom_yaw = 0.0f;
static float s_vx = 0.0f;
static float s_vy = 0.0f;
static float s_wz = 0.0f;
static uint8_t s_mode = MODE_STOP;

static float clampf_local(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void Spec_Init(void) {
    Arm_Init();
    LED_Init();
}

void Spec_SetChassis(float vx, float vy, float wz, uint8_t mode) {
    s_mode = mode;
    if (mode == MODE_STOP || mode == MODE_EMERGENCY) {
        s_vx = 0.0f;
        s_vy = 0.0f;
        s_wz = 0.0f;
        return;
    }
    s_vx = clampf_local(vx, -MAX_VX, MAX_VX);
    s_vy = clampf_local(vy, -MAX_VY, MAX_VY);
    s_wz = clampf_local(wz, -MAX_WZ, MAX_WZ);
}

void Spec_ChassisUpdate(void) {
    float cy = cosf(s_odom_yaw);
    float sy = sinf(s_odom_yaw);
    s_odom_x += (s_vx * cy - s_vy * sy) * CTRL_DT;
    s_odom_y += (s_vx * sy + s_vy * cy) * CTRL_DT;
    s_odom_yaw += s_wz * CTRL_DT;
    if (s_odom_yaw > (float)M_PI) s_odom_yaw -= 2.0f * (float)M_PI;
    if (s_odom_yaw < -(float)M_PI) s_odom_yaw += 2.0f * (float)M_PI;
}

void Spec_GetOdom(float *x, float *y, float *yaw) {
    *x = s_odom_x;
    *y = s_odom_y;
    *yaw = s_odom_yaw;
}

void Spec_GetVelocity(float *vx, float *vy, float *wz) {
    *vx = s_vx;
    *vy = s_vy;
    *wz = s_wz;
}

uint8_t Spec_GetMode(void) {
    return s_mode;
}
