/**
 * main.c  ——  探测先遣机 STM32F407 FreeRTOS
 *
 * 任务:
 *   ChassisTask  1kHz  P5   底盘PID+CAN，含RC接管
 *   ImuTask      500Hz P4   BMI088+Mahony
 *   CommTxTask   50Hz  P3   里程计/IMU → RK3588
 *   WdgTask      10Hz  P1   IWDG喂狗
 *
 * SBUS 遥控: USART1 PA10 100000bps 2E  FrSky X8R
 * 优先级: RC接管 > 上位机指令 > 自主模式
 */
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "swarm_protocol.h"
#include "chassis.h"
#include "comm.h"
#include "motor_m3508.h"
#include "imu_bmi088.h"

/* ── SBUS 状态 ── */
static uint8_t  s_sbus_buf[25];
static uint8_t  s_sbus_idx = 0;
static uint16_t s_rc_ch[16];
static uint8_t  s_rc_ok  = 0;   /* 1=有效RC信号 */
static uint32_t s_rc_tick = 0;

static void SBUS_Parse(const uint8_t *d) {
    if (d[0] != 0x0F || d[24] != 0x00) return;
    s_rc_ch[0] = ((uint16_t)d[1]     | ((uint16_t)d[2] << 8)) & 0x7FF;
    s_rc_ch[1] = ((uint16_t)d[2]>>3  | ((uint16_t)d[3] << 5)) & 0x7FF;
    s_rc_ch[2] = ((uint16_t)d[3]>>6  | ((uint16_t)d[4] << 2)
                                      | ((uint16_t)d[5] <<10)) & 0x7FF;
    s_rc_ch[3] = ((uint16_t)d[5]>>1  | ((uint16_t)d[6] << 7)) & 0x7FF;
    s_rc_ch[4] = ((uint16_t)d[6]>>4  | ((uint16_t)d[7] << 4)) & 0x7FF;
    /* 检测failsafe位 */
    s_rc_ok  = (d[23] & 0x08) ? 0 : 1;
    s_rc_tick = xTaskGetTickCountFromISR();
}

static float RC_Norm(uint16_t v) {
    return ((float)v - 991.5f) / 819.5f;
}

/* ── SBUS USART1 中断 ── */
void USART1_IRQHandler(void) {
    if (!(USART1->SR & USART_SR_RXNE)) return;
    uint8_t b = (uint8_t)(USART1->DR & 0xFF);
    if (s_sbus_idx == 0 && b != 0x0F) return;
    s_sbus_buf[s_sbus_idx++] = b;
    if (s_sbus_idx >= 25) {
        SBUS_Parse(s_sbus_buf);
        s_sbus_idx = 0;
    }
}

/* ── 任务 ── */
static void ChassisTask(void *arg) {
    (void)arg;
    Chassis_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) {
        uint32_t now = xTaskGetTickCount();
        /* RC接管：有效信号且500ms内 */
        if (s_rc_ok && (now - s_rc_tick) < 500U) {
            float vx = RC_Norm(s_rc_ch[1]) * 0.6f;  /* 左摇杆上下 */
            float vy = RC_Norm(s_rc_ch[0]) * 0.6f;  /* 左摇杆左右 */
            float wz = RC_Norm(s_rc_ch[3]) * 2.0f;  /* 右摇杆左右 */
            /* Ch5 开关 < 400 → 紧急停止 */
            if (s_rc_ch[4] < 400) Chassis_SetTarget(0, 0, 0, MODE_EMERGENCY);
            else                   Chassis_SetTarget(vx, vy, wz, MODE_RC);
        }
        Chassis_Update();
        vTaskDelayUntil(&wk, pdMS_TO_TICKS(1));
    }
}

static void ImuTask(void *arg) {
    (void)arg;
    IMU_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) {
        IMU_Update(0.002f);
        vTaskDelayUntil(&wk, pdMS_TO_TICKS(2));
    }
}

static void CommTxTask(void *arg) {
    (void)arg;
    Comm_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) {
        Comm_TxTask();
        vTaskDelayUntil(&wk, pdMS_TO_TICKS(20));
    }
}

static void WdgTask(void *arg) {
    (void)arg;
    TickType_t wk = xTaskGetTickCount();
    while (1) {
        IWDG->KR = 0xAAAAU;
        vTaskDelayUntil(&wk, pdMS_TO_TICKS(100));
    }
}

/* ── 初始化 ── */
static void SBUS_Init(void) {
    /* PA10 RX  AF7  USART1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOA->MODER  &= ~(3U<<20); GPIOA->MODER |= (2U<<20);
    GPIOA->AFR[1] &= ~(0xFU<<8); GPIOA->AFR[1] |= (7U<<8);
    /* 100000bps @ APB2=84MHz BRR=840  2stopbits  EvenParity */
    USART1->BRR  = 840U;
    USART1->CR2  = USART_CR2_STOP_1;
    USART1->CR1  = USART_CR1_UE|USART_CR1_RE|USART_CR1_RXNEIE
                 | USART_CR1_PCE;
    NVIC_SetPriority(USART1_IRQn, 6);
    NVIC_EnableIRQ(USART1_IRQn);
}

static void IWDG_Init(void) {
    IWDG->KR = 0x5555U; IWDG->PR = 3U;
    IWDG->RLR = 1250U;  IWDG->KR = 0xCCCCU;
}

int main(void) {
    Motor_Init();
    SBUS_Init();
    IWDG_Init();
    xTaskCreate(ChassisTask, "Chassis", 512, NULL, 5, NULL);
    xTaskCreate(ImuTask,     "IMU",     256, NULL, 4, NULL);
    xTaskCreate(CommTxTask,  "CommTx",  256, NULL, 3, NULL);
    xTaskCreate(WdgTask,     "WDG",     128, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *n)
  { (void)t; (void)n; __disable_irq(); while (1); }
void vApplicationMallocFailedHook(void)
  { __disable_irq(); while (1); }
