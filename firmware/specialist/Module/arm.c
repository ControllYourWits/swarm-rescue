/**
 * arm.c -- 4-DOF 机械臂舵机控制
 *
 * TIM3 CH1-5, 50Hz PWM
 *   CH1: 关节 0 (底座旋转)   0-180deg
 *   CH2: 关节 1 (肩部)       0-180deg
 *   CH3: 关节 2 (肘部)       0-180deg
 *   CH4: 关节 3 (腕部)       0-180deg
 *   CH5: 夹爪                0=张开, 1=夹紧
 *
 * 脉宽映射: 500us = 0deg, 2500us = 180deg
 * 关节角度以度为单位, 以 100Hz 插值运动.
 */
#include "arm.h"
#include "stm32f4xx.h"
#include <math.h>

#define JOINT_COUNT   4
#define INTERP_SPEED  3.0f    /* deg per 10ms step */

static float s_target[JOINT_COUNT] = {90.0f, 10.0f, 10.0f, 0.0f};
static float s_current[JOINT_COUNT] = {90.0f, 10.0f, 10.0f, 0.0f};
static uint8_t s_gripper_state = 0;

static void SetServo(uint8_t ch, uint16_t pulse_us) {
    if (pulse_us < 500)  pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    switch (ch) {
    case 0: TIM3->CCR1 = pulse_us; break;
    case 1: TIM3->CCR2 = pulse_us; break;
    case 2: TIM3->CCR3 = pulse_us; break;
    case 3: TIM3->CCR4 = pulse_us; break;
    }
}

static uint16_t DegToPulse(float deg) {
    /* deg clamped [0, 180] */
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    return (uint16_t)(500.0f + deg * (2000.0f / 180.0f));
}

static void SetGripperPulse(uint8_t closed) {
    /* TIM3 CH5 not available on all F4 parts; use TIM3_CH1 extended or
       separate timer.  Here we use a software-controlled output on PC6. */
    if (closed) GPIO_SetBits(GPIOC, GPIO_Pin_6);
    else        GPIO_ResetBits(GPIOC, GPIO_Pin_6);
}

void Arm_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* PA6/PA7/PB0/PB1 -> AF2 (TIM3 CH1-4) */
    GPIOA->MODER &= ~((3U<<12)|(3U<<14));
    GPIOA->MODER |=  (2U<<12)|(2U<<14);
    GPIOA->AFR[0] &= ~((0xFU<<24)|(0xFU<<28));
    GPIOA->AFR[0] |=  (2U<<24)|(2U<<28);

    GPIOB->MODER &= ~((3U<<0)|(3U<<2));
    GPIOB->MODER |=  (2U<<0)|(2U<<2);
    GPIOB->AFR[0] &= ~((0xFU<<0)|(0xFU<<4));
    GPIOB->AFR[0] |=  (2U<<0)|(2U<<4);

    /* PC6 as GPIO output for gripper */
    GPIOC->MODER &= ~(3U<<12); GPIOC->MODER |= (1U<<12);

    /* TIM3: 84MHz / 1680 = 50kHz; ARR=1000 → 50Hz PWM */
    TIM3->PSC = 1680 - 1;
    TIM3->ARR = 1000 - 1;
    TIM3->CCMR1 = (TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE)
                | ((TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2PE) << 8);
    TIM3->CCMR2 = (TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3PE)
                | ((TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4PE) << 8);
    TIM3->CCER = TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;
    TIM3->CR1 |= TIM_CR1_CEN;

    /* Home position */
    for (int i = 0; i < JOINT_COUNT; i++) {
        s_current[i] = (float[]){90.0f, 10.0f, 10.0f, 0.0f}[i];
        s_target[i]  = s_current[i];
        SetServo(i, DegToPulse(s_current[i]));
    }
}

void Arm_SetJoint(uint8_t joint, float angle_deg) {
    if (joint >= JOINT_COUNT) return;
    s_target[joint] = angle_deg;
}

void Arm_SetGripper(uint8_t state) {
    s_gripper_state = state;
    SetGripperPulse(state);
}

void Arm_Update(void) {
    for (int i = 0; i < JOINT_COUNT; i++) {
        float diff = s_target[i] - s_current[i];
        float absd = fabsf(diff);
        if (absd < 0.1f) continue;
        float step = diff > INTERP_SPEED  ? INTERP_SPEED
                   : diff < -INTERP_SPEED ? -INTERP_SPEED
                   : diff;
        s_current[i] += step;
        SetServo(i, DegToPulse(s_current[i]));
    }
}
