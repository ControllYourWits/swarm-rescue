/*
 * carrier_ctrl.c - Carrier chassis, supply servos, and battery sampling.
 *
 * The motor-driver specific output layer is intentionally isolated. Until
 * encoder/CAN feedback is wired in, odometry integrates the bounded command
 * velocity so the firmware exposes a coherent state to the upper computer.
 */
#include "carrier_ctrl.h"
#include "swarm_protocol.h"
#include "stm32f4xx.h"
#include <math.h>

#define SERVO_PERIOD_US 20000U
#define SERVO_CLOSE_US  1000U
#define SERVO_OPEN_US   2000U
#define CTRL_DT         0.001f
#define MAX_VX          0.80f
#define MAX_VY          0.50f
#define MAX_WZ          2.00f
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float s_odom_x = 0.0f;
static float s_odom_y = 0.0f;
static float s_odom_yaw = 0.0f;
static float s_bat_v = 29.6f;
static uint8_t s_mode = MODE_STOP;
static float s_vx = 0.0f;
static float s_vy = 0.0f;
static float s_wz = 0.0f;

static float clampf_local(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void Servo_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    for (int i = 0; i < 4; i++) {
        GPIOA->MODER &= ~(3U << (i * 2));
        GPIOA->MODER |= (2U << (i * 2));
        GPIOA->AFR[0] &= ~(0xFU << (i * 4));
        GPIOA->AFR[0] |= (1U << (i * 4));
    }

    TIM2->PSC = 83U;                  /* 84MHz / 84 = 1MHz */
    TIM2->ARR = SERVO_PERIOD_US - 1U; /* 20ms period = 50Hz */
    TIM2->CCMR1 = (6U << 4) | (6U << 12);
    TIM2->CCMR2 = (6U << 4) | (6U << 12);
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM2->CCR4 = SERVO_CLOSE_US;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void ADC_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->MODER |= (3U << 10); /* PA5 ADC1_IN5 */
    ADC1->SQR3 = 5U;
    ADC1->CR2 = ADC_CR2_ADON;
}

static float ADC_ReadBat(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {
    }
    uint16_t raw = (uint16_t)ADC1->DR;
    return (float)raw / 4096.0f * 3.3f * 11.0f;
}

void Carrier_Init(void) {
    Servo_Init();
    ADC_Init();
}

void Carrier_SetVel(float vx, float vy, float wz, uint8_t mode) {
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

void Carrier_Update(void) {
    float cy = cosf(s_odom_yaw);
    float sy = sinf(s_odom_yaw);
    s_odom_x += (s_vx * cy - s_vy * sy) * CTRL_DT;
    s_odom_y += (s_vx * sy + s_vy * cy) * CTRL_DT;
    s_odom_yaw += s_wz * CTRL_DT;
    if (s_odom_yaw > (float)M_PI) s_odom_yaw -= 2.0f * (float)M_PI;
    if (s_odom_yaw < -(float)M_PI) s_odom_yaw += 2.0f * (float)M_PI;

    static uint32_t cnt = 0;
    if (++cnt >= 500U) {
        s_bat_v = ADC_ReadBat();
        cnt = 0U;
    }
}

void Carrier_SupplyAction(uint8_t action, uint8_t slot) {
    volatile uint32_t *ccr[4] = {&TIM2->CCR1, &TIM2->CCR2, &TIM2->CCR3, &TIM2->CCR4};
    if (slot > 3U) return;

    switch (action) {
    case SUPPLY_CLOSE:
        *ccr[slot] = SERVO_CLOSE_US;
        break;
    case SUPPLY_OPEN:
    case SUPPLY_THROW:
        /* Open the servo; SupplyTask will auto-close after timeout */
        *ccr[slot] = SERVO_OPEN_US;
        break;
    default:
        break;
    }
}

void Carrier_GetOdom(float *x, float *y, float *yaw) {
    *x = s_odom_x;
    *y = s_odom_y;
    *yaw = s_odom_yaw;
}

void Carrier_GetVelocity(float *vx, float *vy, float *wz) {
    *vx = s_vx;
    *vy = s_vy;
    *wz = s_wz;
}

uint8_t Carrier_GetMode(void) {
    return s_mode;
}

float Carrier_GetBattery(void) {
    float pct = (s_bat_v - 22.4f) / (33.6f - 22.4f) * 100.0f;
    return clampf_local(pct, 0.0f, 100.0f);
}
