#include "chassis.h"
#include "pid.h"
#include "kinematics.h"
#include "motor_m3508.h"
#include "imu_bmi088.h"
#include "swarm_protocol.h"
#include <math.h>

#define DT           0.001f
#define MAX_CUR      16384.0f
#define BAT_V        24.0f
#define DEF_PWR      60.0f   /* 废墟中功率降低保护电机 */
#define SPIN_WZ      4.71f   /* 小陀螺 1.5π rad/s */

static PID_t   s_pid[MOTOR_NUM];
static float   s_rpm_set[MOTOR_NUM];
static float   s_cur[MOTOR_NUM];
static uint8_t s_mode   = MODE_STOP;
static float   s_vx=0, s_vy=0, s_wz=0;
static float   s_pwr    = DEF_PWR;
static float   s_odom_x=0, s_odom_y=0, s_odom_yaw=0;
static float   s_vx_a=0, s_vy_a=0, s_wz_a=0;

void Chassis_Init(void) {
    for (int i = 0; i < MOTOR_NUM; i++)
        PID_Init(&s_pid[i], 12.0f, 0.8f, 0.0f, 2000.f, MAX_CUR);
}

void Chassis_SetTarget(float vx, float vy, float wz, uint8_t mode) {
    s_mode = mode;
    s_vx   = vx; s_vy = vy;
    s_wz   = (mode == MODE_STOP) ? 0.0f : wz;
}

void Chassis_SetPwrLimit(float w) { s_pwr = w; }

static void UpdateOdom(void) {
    float rpm[MOTOR_NUM];
    for (int i=0;i<MOTOR_NUM;i++) rpm[i]=(float)g_motor[i].speed_rpm;
    Kin_Forward(rpm, &s_vx_a, &s_vy_a, &s_wz_a);
    s_odom_yaw = g_ahrs.yaw;
    float cy = cosf(s_odom_yaw), sy = sinf(s_odom_yaw);
    s_odom_x += (s_vx_a*cy - s_vy_a*sy) * DT;
    s_odom_y += (s_vx_a*sy + s_vy_a*cy) * DT;
}

static void PowerLimit(void) {
    float p = 0;
    for (int i=0;i<MOTOR_NUM;i++)
        p += fabsf(s_cur[i]) / MAX_CUR * 20.0f * BAT_V;
    if (p > s_pwr && p > 0.1f) {
        float sc = s_pwr / p;
        for (int i=0;i<MOTOR_NUM;i++) s_cur[i] *= sc;
    }
}

void Chassis_Update(void) {
    UpdateOdom();
    Motor_OfflineCheck();
    if (s_mode == MODE_STOP || s_mode == MODE_EMERGENCY) {
        Motor_Send(0,0,0,0);
        for (int i=0;i<MOTOR_NUM;i++) PID_Reset(&s_pid[i]);
        return;
    }
    Kin_Inverse(s_vx, s_vy, s_wz, s_rpm_set);
    for (int i=0;i<MOTOR_NUM;i++)
        s_cur[i] = PID_Calc(&s_pid[i], s_rpm_set[i],
                             (float)g_motor[i].speed_rpm, DT);
    PowerLimit();
    Motor_Send((int16_t)s_cur[0],(int16_t)s_cur[1],
               (int16_t)s_cur[2],(int16_t)s_cur[3]);
}

void Chassis_GetActual(float *vx, float *vy, float *wz)
  { *vx=s_vx_a; *vy=s_vy_a; *wz=s_wz_a; }
void Chassis_GetOdom(float *x, float *y, float *yaw)
  { *x=s_odom_x; *y=s_odom_y; *yaw=s_odom_yaw; }
