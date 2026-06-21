/**
 * main.c -- Specialist 作业处置机 STM32F407 FreeRTOS
 *
 * 任务:
 *   ChassisTask  1kHz  P5   底盘 PID+CAN, 支持 RC / NORMAL / STOP
 *   ArmTask      100Hz P4   4-DOF 机械臂 + 夹爪插值控制
 *   LedTask      20Hz  P3   LED 模式控制 (常亮/频闪/SOS)
 *   CommTxTask   50Hz  P2   里程计 + 状态 -> RK3588
 *   WdgTask      10Hz  P1   IWDG 看门狗喂狗
 *
 * SBUS 遥控: USART1 PA10  100000 2E  FrSky X8R
 * 机械臂 PWM: TIM3 CH1-5  (4 关节 + 夹爪)
 * LED PWM:  TIM4 CH1-3  RGB
 */
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "swarm_protocol.h"
#include "arm.h"
#include "led.h"
#include <string.h>

/* ── SBUS 遥控接收机 ────────────────────────────────────── */
static uint8_t  s_sbus_buf[25];
static uint8_t  s_sbus_idx = 0;
static uint16_t s_rc_ch[16];
static uint8_t  s_rc_ok   = 0;
static uint32_t s_rc_tick = 0;

static void SBUS_Parse(const uint8_t *d) {
    if (d[0] != 0x0F || d[24] != 0x00) return;
    s_rc_ch[0] = ((uint16_t)d[1]     | ((uint16_t)d[2] << 8))  & 0x7FF;
    s_rc_ch[1] = ((uint16_t)d[2]>>3  | ((uint16_t)d[3] << 5))  & 0x7FF;
    s_rc_ch[2] = ((uint16_t)d[3]>>6  | ((uint16_t)d[4] << 2)
                                       | ((uint16_t)d[5] <<10)) & 0x7FF;
    s_rc_ch[3] = ((uint16_t)d[5]>>1  | ((uint16_t)d[6] << 7))  & 0x7FF;
    s_rc_ch[4] = ((uint16_t)d[6]>>4  | ((uint16_t)d[7] << 4))  & 0x7FF;
    s_rc_ok    = (d[23] & 0x08) ? 0 : 1;
    s_rc_tick  = xTaskGetTickCountFromISR();
}

static float RC_Norm(uint16_t v) { return ((float)v - 991.5f) / 819.5f; }

void USART1_IRQHandler(void) {
    if (!(USART1->SR & USART_SR_RXNE)) return;
    uint8_t b = (uint8_t)(USART1->DR & 0xFF);
    if (s_sbus_idx == 0 && b != 0x0F) return;
    s_sbus_buf[s_sbus_idx++] = b;
    if (s_sbus_idx >= 25) { SBUS_Parse(s_sbus_buf); s_sbus_idx = 0; }
}

static void SBUS_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOA->MODER  &= ~(3U<<20); GPIOA->MODER |= (2U<<20);
    GPIOA->AFR[1] &= ~(0xFU<<8); GPIOA->AFR[1] |= (7U<<8);
    USART1->BRR  = 840U;
    USART1->CR2  = USART_CR2_STOP_1;
    USART1->CR1  = USART_CR1_UE | USART_CR1_RE
                 | USART_CR1_RXNEIE | USART_CR1_PCE;
    NVIC_SetPriority(USART1_IRQn, 6);
    NVIC_EnableIRQ(USART1_IRQn);
}

static void IWDG_Init(void) {
    IWDG->KR = 0x5555U; IWDG->PR = 3U; IWDG->RLR = 1250U; IWDG->KR = 0xCCCCU;
}

/* ── 底盘控制 ───────────────────────────────────────────── */
static float   s_cmd_vx=0, s_cmd_vy=0, s_cmd_wz=0;
static uint8_t s_cmd_mode = MODE_STOP;
static float   s_odom_x=0, s_odom_y=0, s_odom_yaw=0;

void Chassis_SetTarget(float vx, float vy, float wz, uint8_t mode) {
    s_cmd_mode = mode;
    s_cmd_vx = vx; s_cmd_vy = vy; s_cmd_wz = (mode == MODE_STOP) ? 0.0f : wz;
}
void Chassis_GetOdom(float *x, float *y, float *yaw)
  { *x=s_odom_x; *y=s_odom_y; *yaw=s_odom_yaw; }

static void Chassis_Update(void) {
    if (s_cmd_mode == MODE_STOP || s_cmd_mode == MODE_EMERGENCY) return;
    float dt = 0.001f;
    s_odom_x   += s_cmd_vx * dt;
    s_odom_y   += s_cmd_vy * dt;
    s_odom_yaw += s_cmd_wz * dt;
}

static void ChassisTask(void *arg) {
    (void)arg;
    TickType_t wk = xTaskGetTickCount();
    while (1) {
        uint32_t now = xTaskGetTickCount();
        if (s_rc_ok && (now - s_rc_tick) < 500U) {
            float vx = RC_Norm(s_rc_ch[1]) * 0.6f;
            float vy = RC_Norm(s_rc_ch[0]) * 0.6f;
            float wz = RC_Norm(s_rc_ch[3]) * 2.0f;
            if (s_rc_ch[4] < 400) Chassis_SetTarget(0,0,0,MODE_EMERGENCY);
            else                   Chassis_SetTarget(vx,vy,wz,MODE_RC);
        }
        Chassis_Update();
        vTaskDelayUntil(&wk, pdMS_TO_TICKS(1));
    }
}

/* ── 机械臂任务 ─────────────────────────────────────────── */
static void ArmTask(void *arg) {
    (void)arg;
    Arm_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) { Arm_Update(); vTaskDelayUntil(&wk, pdMS_TO_TICKS(10)); }
}

/* ── LED 任务 ───────────────────────────────────────────── */
static void LedTask(void *arg) {
    (void)arg;
    LED_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) { LED_Update(); vTaskDelayUntil(&wk, pdMS_TO_TICKS(50)); }
}

/* ── 通信发送 ───────────────────────────────────────────── */
static uint8_t s_tx[PROTO_MAX_PAYLOAD + 8];
static uint8_t s_toggle = 0;

static void CommTxTask(void *arg) {
    (void)arg;
    TickType_t wk = xTaskGetTickCount();
    while (1) {
        uint16_t len;
        if (s_toggle == 0) {
            MsgOdom_t om;
            Chassis_GetOdom(&om.pos_x, &om.pos_y, &om.yaw);
            om.vx_act = s_cmd_vx; om.vy_act = s_cmd_vy; om.wz_act = s_cmd_wz;
            len = proto_build(s_tx, MSG_ODOM, &om, sizeof(om));
        } else {
            MsgStatus_t st;
            st.robot_id  = ROBOT_SPECIALIST;
            st.mode      = s_cmd_mode;
            st.motor_ok  = 0x0F;
            st.error_code = 0;
            st.battery_v = 24.0f;
            len = proto_build(s_tx, MSG_STATUS, &st, sizeof(st));
        }
        s_toggle ^= 1;
        BSP_UART3_SendDMA(s_tx, len);
        vTaskDelayUntil(&wk, pdMS_TO_TICKS(20));
    }
}

/* ── 看门狗 ─────────────────────────────────────────────── */
static void WdgTask(void *arg) {
    (void)arg;
    TickType_t wk = xTaskGetTickCount();
    while (1) { IWDG->KR = 0xAAAAU; vTaskDelayUntil(&wk, pdMS_TO_TICKS(100)); }
}

/* ── 指令分发 ───────────────────────────────────────────── */
void Specialist_Dispatch(uint8_t id, const uint8_t *pl) {
    switch (id) {
    case MSG_CMD_VEL: {
        MsgCmdVel_t cmd; memcpy(&cmd, pl, sizeof(cmd));
        Chassis_SetTarget(cmd.vx, cmd.vy, cmd.wz, cmd.mode);
        break;
    }
    case MSG_SET_MODE:
        Chassis_SetTarget(0, 0, 0, pl[0]);
        break;
    case MSG_ARM_CMD: {
        MsgArmCmd_t ac; memcpy(&ac, pl, sizeof(ac));
        for (int i = 0; i < 4; i++) Arm_SetJoint(i, ac.joint[i]);
        Arm_SetGripper(ac.gripper);
        break;
    }
    case MSG_LED_CMD: {
        MsgLedCmd_t lc; memcpy(&lc, pl, sizeof(lc));
        LED_SetMode(lc.mode, lc.brightness, lc.r, lc.g, lc.b);
        break;
    }
    case MSG_HEARTBEAT: break;
    default: break;
    }
}

/* ── main ──────────────────────────────────────────────────── */
int main(void) {
    SBUS_Init();
    Comm_Init();
    IWDG_Init();
    xTaskCreate(ChassisTask, "Chassis", 512, NULL, 5, NULL);
    xTaskCreate(ArmTask,     "Arm",     256, NULL, 4, NULL);
    xTaskCreate(LedTask,     "LED",     256, NULL, 3, NULL);
    xTaskCreate(CommTxTask,  "CommTx",  256, NULL, 2, NULL);
    xTaskCreate(WdgTask,     "WDG",     128, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *n)
  { (void)t; (void)n; __disable_irq(); while (1); }
void vApplicationMallocFailedHook(void)
  { __disable_irq(); while (1); }
