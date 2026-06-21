#pragma once
/**
 * @file    swarm_protocol.h
 * @brief   三机协同通信协议（STM32 与 RK3588 双端共用）
 *
 * 帧格式: [0xAA][LEN][MSG_ID][PAYLOAD...][CRC8][0x55]
 * LEN = sizeof(PAYLOAD) + 1
 *
 * 机器人 ID:
 *   0x01 = Scout     探测先遣机
 *   0x02 = Carrier   补给运输机
 *   0x03 = Specialist 作业处置机
 */
#include <stdint.h>

/* ─── 帧定界符 ─── */
#define PROTO_HEADER        0xAA
#define PROTO_TAIL          0x55
#define PROTO_MAX_PAYLOAD   80

/* ─── 机器人 ID ─── */
#define ROBOT_SCOUT         0x01
#define ROBOT_CARRIER       0x02
#define ROBOT_SPECIALIST    0x03

/* ─── 消息 ID（上行：STM32→RK3588） ─── */
#define MSG_ODOM            0x01   /* 里程计 */
#define MSG_IMU             0x02   /* IMU 原始数据 */
#define MSG_STATUS          0x03   /* 机器人状态 */
#define MSG_BATTERY         0x04   /* 电池信息 */

/* ─── 消息 ID（下行：RK3588→STM32） ─── */
#define MSG_CMD_VEL         0x10   /* 速度指令 */
#define MSG_SET_MODE        0x11   /* 模式切换 */
#define MSG_ARM_CMD         0x12   /* 机械臂指令（Specialist专用） */
#define MSG_SUPPLY_CMD      0x13   /* 投送指令（Carrier专用） */
#define MSG_LED_CMD         0x14   /* LED指令（Specialist专用） */
#define MSG_HEARTBEAT       0x1F   /* 心跳 */

/* ─── 底盘模式 ─── */
#define MODE_STOP           0x00
#define MODE_NORMAL         0x01
#define MODE_FOLLOW         0x02   /* Carrier跟随模式 */
#define MODE_RC             0x03   /* 遥控接管模式 */
#define MODE_EMERGENCY      0xFF   /* 紧急停止 */

/* ─── 结构体定义 ─── */

/* 速度指令 (16B) */
typedef struct __attribute__((packed)) {
    float   vx;       /* m/s 前进   */
    float   vy;       /* m/s 横移   */
    float   wz;       /* rad/s 旋转 */
    uint8_t mode;
    uint8_t _pad[3];
} MsgCmdVel_t;

/* 里程计 (24B) */
typedef struct __attribute__((packed)) {
    float vx_act, vy_act, wz_act;
    float pos_x, pos_y, yaw;
} MsgOdom_t;

/* IMU (36B) */
typedef struct __attribute__((packed)) {
    float accel[3];
    float gyro[3];
    float euler[3];   /* roll pitch yaw rad */
} MsgImu_t;

/* 机器人状态 (8B) */
typedef struct __attribute__((packed)) {
    uint8_t robot_id;
    uint8_t mode;
    uint8_t motor_ok;    /* bit0~3 */
    uint8_t error_code;
    float   battery_v;
} MsgStatus_t;

/* 电池信息 (12B) */
typedef struct __attribute__((packed)) {
    float   voltage;     /* V */
    float   current;     /* A */
    uint8_t percent;     /* 0~100 */
    uint8_t charging;
    uint8_t _pad[2];
} MsgBattery_t;

/* 机械臂指令 — Specialist (20B) */
typedef struct __attribute__((packed)) {
    float   joint[4];    /* 4个关节角度 rad */
    uint8_t gripper;     /* 0=松开 1=夹紧 */
    uint8_t mode;        /* 0=位置 1=速度 */
    uint8_t _pad[2];
} MsgArmCmd_t;

/* 投送指令 — Carrier (4B) */
typedef struct __attribute__((packed)) {
    uint8_t action;      /* 0=关仓 1=开仓 2=抛投 */
    uint8_t slot;        /* 0~3 仓位编号 */
    uint8_t _pad[2];
} MsgSupplyCmd_t;

/* LED指令 — Specialist (8B) */
typedef struct __attribute__((packed)) {
    uint8_t mode;        /* 0=关 1=常亮 2=频闪 3=SOS */
    uint8_t brightness;  /* 0~255 */
    uint8_t r, g, b;
    uint8_t _pad[3];
} MsgLedCmd_t;

/* ─── CRC8 (多项式 0x07) ─── */
static inline uint8_t proto_crc8(const uint8_t *d, uint16_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *d++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ─── 封帧 ─── */
static inline uint16_t proto_build(uint8_t *buf, uint8_t id,
                                    const void *pl, uint8_t plen) {
    buf[0] = PROTO_HEADER;
    buf[1] = (uint8_t)(plen + 1);
    buf[2] = id;
    for (uint8_t i = 0; i < plen; i++)
        buf[3 + i] = ((const uint8_t *)pl)[i];
    buf[3 + plen] = proto_crc8(&buf[1], (uint16_t)(plen + 2));
    buf[4 + plen] = PROTO_TAIL;
    return (uint16_t)(5 + plen);
}
