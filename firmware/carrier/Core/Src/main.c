/**
 * main.c -- Carrier 补给运输机 STM32F407 FreeRTOS
 *
 * 任务:
 *   ChassisTask  1kHz  P5   底盘控制 (调用 carrier_ctrl 模块)
 *   SupplyTask   50Hz  P4   舵机投送控制 (4 槽位)
 *   CommTxTask   50Hz  P3   里程计 + 电池信息 -> 上位机
 *   WdgTask      10Hz  P1   IWDG 看门狗喂狗
 *
 * 底盘/通信逻辑委托给 Module/carrier_ctrl.c 和 Module/comm.c
 * SBUS 遥控: USART1 PA10  100000 2E  FrSky X8R
 */
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "swarm_protocol.h"
#include "carrier_ctrl.h"
#include "comm.h"
#include "supply.h"
#include <string.h>

/* ── SBUS 遥控 ─────────────────────────────────────────── */
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

/* ── FreeRTOS 任务 ─────────────────────────────────────── */

static void ChassisTask(void *arg) {
    (void)arg;
    Carrier_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) {
        uint32_t now = xTaskGetTickCount();
        if (s_rc_ok && (now - s_rc_tick) < 500U) {
            float vx = RC_Norm(s_rc_ch[1]) * 0.6f;
            float vy = RC_Norm(s_rc_ch[0]) * 0.6f;
            float wz = RC_Norm(s_rc_ch[3]) * 1.5f;
            if (s_rc_ch[4] < 400) Carrier_SetVel(0, 0, 0, MODE_EMERGENCY);
            else                   Carrier_SetVel(vx, vy, wz, MODE_RC);
        }
        Carrier_Update();
        vTaskDelayUntil(&wk, pdMS_TO_TICKS(1));
    }
}

static void SupplyTask(void *arg) {
    (void)arg;
    Supply_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) { Supply_Update(); vTaskDelayUntil(&wk, pdMS_TO_TICKS(20)); }
}

static void CommTxTask(void *arg) {
    (void)arg;
    Comm_Init();
    TickType_t wk = xTaskGetTickCount();
    while (1) { Comm_TxTask(); vTaskDelayUntil(&wk, pdMS_TO_TICKS(20)); }
}

static void WdgTask(void *arg) {
    (void)arg;
    TickType_t wk = xTaskGetTickCount();
    while (1) { IWDG->KR = 0xAAAAU; vTaskDelayUntil(&wk, pdMS_TO_TICKS(100)); }
}

/* ── main ──────────────────────────────────────────────── */
int main(void) {
    SBUS_Init();
    IWDG_Init();
    xTaskCreate(ChassisTask, "Chassis", 512, NULL, 5, NULL);
    xTaskCreate(SupplyTask,  "Supply",  256, NULL, 4, NULL);
    xTaskCreate(CommTxTask,  "CommTx",  256, NULL, 3, NULL);
    xTaskCreate(WdgTask,     "WDG",     128, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *n)
  { (void)t; (void)n; __disable_irq(); while (1); }
void vApplicationMallocFailedHook(void)
  { __disable_irq(); while (1); }
