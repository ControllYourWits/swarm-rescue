/**
 * supply.c -- 物资投送舵机控制
 *
 * 4 槽位 PWM 舵机: TIM3 CH1-4, 50Hz, 脉宽 500-2500us
 *   槽位 0: 饮用水   (PA6 TIM3_CH1)
 *   槽位 1: 急救包   (PA7 TIM3_CH2)
 *   槽位 2: 备用电池 (PB0 TIM3_CH3)
 *   槽位 3: 绳索     (PB1 TIM3_CH4)
 *
 * 状态机: IDLE -> OPENING -> OPEN -> (3s 超时) -> CLOSING -> CLOSED
 *   开仓: 180deg -> 2500us pulse
 *   关仓:   0deg ->  500us pulse
 */
#include "supply.h"
#include "stm32f4xx.h"

typedef enum { S_IDLE, S_OPENING, S_OPEN, S_CLOSING } SupplyState_t;

static SupplyState_t s_state[SUPPLY_SLOT_COUNT];
static uint32_t      s_open_ticks[SUPPLY_SLOT_COUNT];
#define AUTO_CLOSE_MS  3000U

static void SetServoPulse(uint8_t ch, uint16_t pulse_us) {
    /* pulse_us clamped to [500, 2500] for standard servo */
    if (pulse_us < 500)  pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    switch (ch) {
    case 0: TIM3->CCR1 = pulse_us; break;
    case 1: TIM3->CCR2 = pulse_us; break;
    case 2: TIM3->CCR3 = pulse_us; break;
    case 3: TIM3->CCR4 = pulse_us; break;
    }
}

void Supply_Init(void) {
    /* GPIO: PA6/PA7 AF2 (TIM3), PB0/PB1 AF2 (TIM3) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOA->MODER  &= ~((3U<<12)|(3U<<14));
    GPIOA->MODER  |=  (2U<<12) | (2U<<14);   /* PA6,PA7 AF */
    GPIOA->AFR[0] &= ~((0xFU<<24)|(0xFU<<28));
    GPIOA->AFR[0] |=  (2U<<24) | (2U<<28);   /* AF2 */

    GPIOB->MODER  &= ~((3U<<0)|(3U<<2));
    GPIOB->MODER  |=  (2U<<0) | (2U<<2);     /* PB0,PB1 AF */
    GPIOB->AFR[0] &= ~((0xFU<<0)|(0xFU<<4));
    GPIOB->AFR[0] |=  (2U<<0) | (2U<<4);     /* AF2 */

    /* TIM3: 84MHz, prescaler=1680 → 50kHz tick, ARR=1000 → 50Hz */
    TIM3->PSC  = 1680 - 1;
    TIM3->ARR  = 1000 - 1;
    TIM3->CCMR1 = (TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE)
                | ((TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2PE) << 8);
    TIM3->CCMR2 = (TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3PE)
                | ((TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4PE) << 8);
    TIM3->CCER  = TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;
    TIM3->CR1  |= TIM_CR1_CEN;

    for (int i = 0; i < SUPPLY_SLOT_COUNT; i++) {
        s_state[i] = S_IDLE;
        SetServoPulse(i, 500);   /* 0° = closed */
    }
}

void Supply_Deploy(uint8_t slot) {
    if (slot >= SUPPLY_SLOT_COUNT) return;
    s_state[slot]      = S_OPENING;
    s_open_ticks[slot] = 0;
    SetServoPulse(slot, 2500);   /* 180° = open */
}

void Supply_Close(uint8_t slot) {
    if (slot >= SUPPLY_SLOT_COUNT) return;
    s_state[slot] = S_CLOSING;
    SetServoPulse(slot, 500);
}

void Supply_Update(void) {
    for (int i = 0; i < SUPPLY_SLOT_COUNT; i++) {
        if (s_state[i] == S_OPENING) {
            s_open_ticks[i] += 20;   /* called at 50Hz → 20ms per tick */
            if (s_open_ticks[i] >= AUTO_CLOSE_MS) {
                /* Hold-open timeout reached; close the servo */
                SetServoPulse(i, 500);
                s_state[i] = S_CLOSING;
            }
        } else if (s_state[i] == S_CLOSING) {
            /* Transition to idle once closed */
            s_state[i] = S_IDLE;
        }
    }
}
