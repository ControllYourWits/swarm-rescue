/**
 * imu_bmi088.c
 * SPI1: PA5=SCK PA6=MISO PA7=MOSI  PB0=CS_ACC  PB1=CS_GYRO
 * ACC ±6g 400Hz | GYRO 500dps 400Hz
 */
#include "imu_bmi088.h"
#include "stm32f4xx.h"
#include <math.h>

RawImu_t g_imu_raw;
Mahony_t g_ahrs;

#define ACC_LSB   (2.0f*6.0f*9.80665f/65536.0f)
#define GYRO_LSB  (2.0f*500.0f*(3.14159265f/180.0f)/65536.0f)

static inline void ACC_CS_L(void)  { GPIOB->BSRR = (1U<<16); }
static inline void ACC_CS_H(void)  { GPIOB->BSRR = (1U<<0);  }
static inline void GYRO_CS_L(void) { GPIOB->BSRR = (1U<<17); }
static inline void GYRO_CS_H(void) { GPIOB->BSRR = (1U<<1);  }

static uint8_t SPI1_Byte(uint8_t tx) {
    while (!(SPI1->SR & SPI_SR_TXE)); SPI1->DR = tx;
    while (!(SPI1->SR & SPI_SR_RXNE)); return (uint8_t)SPI1->DR;
}
static void ACC_WR(uint8_t r, uint8_t v) {
    ACC_CS_L(); SPI1_Byte(r&0x7F); SPI1_Byte(v); ACC_CS_H();
}
static void ACC_RD(uint8_t r, uint8_t *b, uint8_t n) {
    ACC_CS_L(); SPI1_Byte(r|0x80); SPI1_Byte(0);
    for (uint8_t i=0;i<n;i++) b[i]=SPI1_Byte(0); ACC_CS_H();
}
static void GYRO_WR(uint8_t r, uint8_t v) {
    GYRO_CS_L(); SPI1_Byte(r&0x7F); SPI1_Byte(v); GYRO_CS_H();
}
static void GYRO_RD(uint8_t r, uint8_t *b, uint8_t n) {
    GYRO_CS_L(); SPI1_Byte(r|0x80);
    for (uint8_t i=0;i<n;i++) b[i]=SPI1_Byte(0); GYRO_CS_H();
}

void IMU_Init(void) {
    /* 时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN;
    /* PA5/6/7 AF5 */
    GPIOA->MODER  &= ~((3U<<10)|(3U<<12)|(3U<<14));
    GPIOA->MODER  |=  ((2U<<10)|(2U<<12)|(2U<<14));
    GPIOA->OSPEEDR|=  ((3U<<10)|(3U<<12)|(3U<<14));
    GPIOA->AFR[0] &= ~((0xFU<<20)|(0xFU<<24)|(0xFU<<28));
    GPIOA->AFR[0] |=  ((5U<<20) |(5U<<24) |(5U<<28));
    /* PB0/1 CS output high */
    GPIOB->MODER &= ~((3U<<0)|(3U<<2));
    GPIOB->MODER |=  ((1U<<0)|(1U<<2));
    GPIOB->BSRR   = (1U<<0)|(1U<<1);
    /* SPI1 master CPOL0 CPHA0 fPCLK/8 */
    SPI1->CR1 = SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|(2U<<3);
    SPI1->CR1|= SPI_CR1_SPE;
    for (volatile int i=0;i<100000;i++);
    /* ACC 初始化 */
    ACC_WR(0x7D, 0x04); for (volatile int i=0;i<50000;i++);
    ACC_WR(0x7C, 0x00); ACC_WR(0x40, 0xA8); ACC_WR(0x41, 0x01);
    /* GYRO 初始化 */
    GYRO_WR(0x0F, 0x01); GYRO_WR(0x10, 0x03);
    Mahony_Init(&g_ahrs);
}

void IMU_Update(float dt) {
    uint8_t b[6];
    /* 加速度 */
    ACC_RD(0x12, b, 6);
    int16_t ax=(int16_t)((b[1]<<4)|(b[0]>>4)); if(ax&0x800) ax|=(int16_t)0xF000;
    int16_t ay=(int16_t)((b[3]<<4)|(b[2]>>4)); if(ay&0x800) ay|=(int16_t)0xF000;
    int16_t az=(int16_t)((b[5]<<4)|(b[4]>>4)); if(az&0x800) az|=(int16_t)0xF000;
    g_imu_raw.accel[0]=(float)ax*ACC_LSB;
    g_imu_raw.accel[1]=(float)ay*ACC_LSB;
    g_imu_raw.accel[2]=(float)az*ACC_LSB;
    /* 陀螺仪 */
    GYRO_RD(0x02, b, 6);
    g_imu_raw.gyro[0]=(float)(int16_t)((b[1]<<8)|b[0])*GYRO_LSB;
    g_imu_raw.gyro[1]=(float)(int16_t)((b[3]<<8)|b[2])*GYRO_LSB;
    g_imu_raw.gyro[2]=(float)(int16_t)((b[5]<<8)|b[4])*GYRO_LSB;
    Mahony_Update(&g_ahrs,
        g_imu_raw.gyro[0], g_imu_raw.gyro[1], g_imu_raw.gyro[2],
        g_imu_raw.accel[0],g_imu_raw.accel[1],g_imu_raw.accel[2], dt);
}
