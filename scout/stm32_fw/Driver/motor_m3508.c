#include "motor_m3508.h"
#include "bsp_can.h"
#include <string.h>

MotorFB_t g_motor[MOTOR_NUM];
static uint16_t s_off[MOTOR_NUM];
#define OFF_MS 100

void Motor_Init(void) {
    memset(g_motor, 0, sizeof(g_motor));
    BSP_CAN1_Init();
}

void Motor_Send(int16_t i1, int16_t i2, int16_t i3, int16_t i4) {
    uint8_t d[8] = {
        (uint8_t)(i1>>8),(uint8_t)i1,
        (uint8_t)(i2>>8),(uint8_t)i2,
        (uint8_t)(i3>>8),(uint8_t)i3,
        (uint8_t)(i4>>8),(uint8_t)i4
    };
    BSP_CAN1_Send(0x200U, d, 8U);
}

void Motor_Parse(uint32_t id, const uint8_t *d) {
    if (id < 0x201 || id > 0x204) return;
    uint8_t i = (uint8_t)(id - 0x201);
    g_motor[i].angle     = (int16_t)((d[0]<<8)|d[1]);
    g_motor[i].speed_rpm = (int16_t)((d[2]<<8)|d[3]);
    g_motor[i].torque    = (int16_t)((d[4]<<8)|d[5]);
    g_motor[i].temp      = d[6];
    g_motor[i].online    = 1;
    s_off[i] = 0;
}

void Motor_OfflineCheck(void) {
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (++s_off[i] > OFF_MS) {
            g_motor[i].online = 0;
            g_motor[i].speed_rpm = 0;
            s_off[i] = OFF_MS;
        }
    }
}

/* CAN 接收回调 */
void BSP_CAN1_RxCB(uint32_t id, const uint8_t *data, uint8_t len) {
    (void)len;
    Motor_Parse(id, data);
}
