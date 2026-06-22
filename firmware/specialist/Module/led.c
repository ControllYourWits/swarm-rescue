/**
 * led.c -- RGB LED 阵列控制
 *
 * TIM4 CH1-3 PWM, 引脚 PB6/PB7/PB8 (AF2)
 *   50kHz PWM, 8-bit 分辨率 -> ARR=255
 *
 * 模式:
 *   0 = 关闭 (所有通道输出 0)
 *   1 = 常亮 (r,g,b) 按亮度缩放
 *   2 = 5Hz 频闪 (50% 占空比, 100ms 亮 / 100ms 灭)
 *   3 = SOS 国际求救信号 (...---...)
 *       点 150ms, 划 450ms, 符号间隔 150ms, 字母间隔 450ms
 */
#include "led.h"
#include "stm32f4xx.h"

static uint8_t s_mode       = LED_MODE_OFF;
static uint8_t s_brightness = 255;
static uint8_t s_r=0, s_g=0, s_b=0;

static uint32_t s_phase_tick = 0;
static uint8_t  s_visible    = 1;

/* SOS pattern state machine */
static uint8_t  s_sos_seq[]  = {0,0,0, 1,1,1, 0,0,0,  2,2,2}; /* 0=dot,1=dash,2=gap */
static uint8_t  s_sos_idx    = 0;
static uint32_t s_sos_timer  = 0;

void LED_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* PB6/PB7/PB8 AF2 (TIM4 CH1-3) */
    GPIOB->MODER  &= ~((3U<<12)|(3U<<14)|(3U<<16));
    GPIOB->MODER  |=  (2U<<12)|(2U<<14)|(2U<<16);
    GPIOB->AFR[0] &= ~((0xFU<<24)|(0xFU<<28));
    GPIOB->AFR[0] |=  (2U<<24)|(2U<<28);
    GPIOB->AFR[1] &= ~(0xFU<<0);
    GPIOB->AFR[1] |=  (2U<<0);

    /* TIM4: 84MHz / 1 = 84MHz; ARR=255 → ~330kHz → fine for LED */
    TIM4->PSC  = 0;
    TIM4->ARR  = 255;
    TIM4->CCMR1 = (TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE)
                | ((TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2PE) << 8);
    TIM4->CCMR2 = (TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3PE);
    TIM4->CCER  = TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E;
    TIM4->CR1  |= TIM_CR1_CEN;

    TIM4->CCR1 = 0; TIM4->CCR2 = 0; TIM4->CCR3 = 0;
}

void LED_SetMode(uint8_t mode, uint8_t brightness,
                 uint8_t r, uint8_t g, uint8_t b) {
    s_mode       = mode;
    s_brightness = brightness;
    s_r = r; s_g = g; s_b = b;
    s_phase_tick = 0;
    s_sos_idx    = 0;
    s_sos_timer  = 0;
    s_visible    = 1;
}

static void SetRGB(uint8_t r, uint8_t g, uint8_t b) {
    TIM4->CCR1 = r; TIM4->CCR2 = g; TIM4->CCR3 = b;
}

void LED_Update(void) {
    uint8_t scale = s_brightness;
    uint8_t r = (uint8_t)((uint16_t)s_r * scale / 255);
    uint8_t g = (uint8_t)((uint16_t)s_g * scale / 255);
    uint8_t b = (uint8_t)((uint16_t)s_b * scale / 255);

    switch (s_mode) {
    case LED_MODE_OFF:
        SetRGB(0, 0, 0);
        break;

    case LED_MODE_CONSTANT:
        SetRGB(r, g, b);
        break;

    case LED_MODE_STROBE:
        s_phase_tick += 50;  /* 50ms per call */
        if (s_phase_tick >= 200) {  /* 200ms period → 5Hz */
            s_phase_tick = 0;
            s_visible = !s_visible;
        }
        SetRGB(s_visible ? r : 0, s_visible ? g : 0, s_visible ? b : 0);
        break;

    case LED_MODE_SOS:
        s_sos_timer += 50;
        if (s_visible) {
            uint8_t sym = s_sos_seq[s_sos_idx];
            uint32_t duration = sym == 0 ? 150 : sym == 1 ? 450 : 150;
            if (s_sos_timer >= duration) {
                s_visible  = 0;
                s_sos_timer = 0;
            }
        } else {
            uint32_t gap = 150;
            if (s_sos_timer >= gap) {
                s_sos_idx = (s_sos_idx + 1) % 12;
                s_visible  = 1;
                s_sos_timer = 0;
            }
        }
        SetRGB(s_visible ? r : 0, s_visible ? g : 0, s_visible ? b : 0);
        break;

    default:
        SetRGB(0, 0, 0);
        break;
    }
}
