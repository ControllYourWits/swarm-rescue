#include "comm.h"
#include "swarm_protocol.h"
#include "chassis.h"
#include "imu_bmi088.h"
#include "bsp_uart.h"
#include <string.h>

/* ──── 发送缓冲 ──── */
static uint8_t s_tx[PROTO_MAX_PAYLOAD + 8];
static uint8_t s_toggle = 0;

/* ──── 接收状态机 ──── */
typedef enum { PS_HEADER, PS_LEN, PS_DATA, PS_CRC, PS_TAIL } PS_t;
static PS_t    s_ps  = PS_HEADER;
static uint8_t s_buf[PROTO_MAX_PAYLOAD + 4];
static uint8_t s_len = 0, s_pos = 0;

static void Dispatch(uint8_t id, const uint8_t *pl) {
    switch (id) {
    case MSG_CMD_VEL: {
        MsgCmdVel_t cmd;
        memcpy(&cmd, pl, sizeof(cmd));
        Chassis_SetTarget(cmd.vx, cmd.vy, cmd.wz, cmd.mode);
        break;
    }
    case MSG_SET_MODE:
        Chassis_SetTarget(0, 0, 0, pl[0]);
        break;
    case MSG_HEARTBEAT:
        /* 重置看门狗计数 */
        break;
    default: break;
    }
}

/* UART 接收中断调用 */
void BSP_UART3_RxCB(uint8_t b) {
    switch (s_ps) {
    case PS_HEADER: if (b==PROTO_HEADER) s_ps=PS_LEN; break;
    case PS_LEN:
        if (b > PROTO_MAX_PAYLOAD) { s_ps = PS_HEADER; break; }
        s_len=b; s_buf[0]=b; s_pos=1; s_ps=PS_DATA; break;
    case PS_DATA:
        if (s_pos >= sizeof(s_buf)) { s_ps = PS_HEADER; break; }
        s_buf[s_pos++]=b;
        if (s_pos==(uint8_t)(s_len+1)) s_ps=PS_CRC;
        break;
    case PS_CRC:
        s_ps = (proto_crc8(s_buf,(uint16_t)(s_len+1))==b) ? PS_TAIL : PS_HEADER;
        break;
    case PS_TAIL:
        if (b==PROTO_TAIL) Dispatch(s_buf[1], &s_buf[2]);
        s_ps=PS_HEADER;
        break;
    }
}

void Comm_Init(void) { BSP_UART3_Init(); }

void Comm_TxTask(void) {
    uint16_t len;
    if (s_toggle == 0) {
        MsgOdom_t om;
        Chassis_GetActual(&om.vx_act, &om.vy_act, &om.wz_act);
        Chassis_GetOdom  (&om.pos_x,  &om.pos_y,  &om.yaw);
        len = proto_build(s_tx, MSG_ODOM, &om, sizeof(om));
    } else {
        MsgImu_t im;
        im.accel[0]=g_imu_raw.accel[0]; im.accel[1]=g_imu_raw.accel[1]; im.accel[2]=g_imu_raw.accel[2];
        im.gyro[0] =g_imu_raw.gyro[0];  im.gyro[1] =g_imu_raw.gyro[1];  im.gyro[2] =g_imu_raw.gyro[2];
        im.euler[0]=g_ahrs.roll; im.euler[1]=g_ahrs.pitch; im.euler[2]=g_ahrs.yaw;
        len = proto_build(s_tx, MSG_IMU, &im, sizeof(im));
    }
    s_toggle ^= 1;
    BSP_UART3_SendDMA(s_tx, len);
}
