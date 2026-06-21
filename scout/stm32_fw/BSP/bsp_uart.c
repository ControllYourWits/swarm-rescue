/**
 * bsp_uart.c — USART3 921600bps  DMA1 Stream3 Ch4 发送
 * PB10=TX  PB11=RX  AF7
 */
#include "bsp_uart.h"
#include "stm32f4xx.h"

void BSP_UART3_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    /* PB10 TX  PB11 RX  AF7 */
    GPIOB->MODER  &= ~((3U<<20)|(3U<<22));
    GPIOB->MODER  |=  ((2U<<20)|(2U<<22));
    GPIOB->OSPEEDR|=  ((3U<<20)|(3U<<22));
    GPIOB->AFR[1] &= ~((0xFU<<8)|(0xFU<<12));
    GPIOB->AFR[1] |=  ((7U<<8) |(7U<<12));
    /* USART3  921600 @ 42MHz → BRR=0x02D9 */
    USART3->BRR  = 0x02D9U;
    USART3->CR1  = USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
    USART3->CR3 |= USART_CR3_DMAT;
    /* DMA1 Stream3 Ch4 → USART3 TX */
    DMA1_Stream3->CR = 0;
    DMA1_Stream3->PAR = (uint32_t)&USART3->DR;
    DMA1_Stream3->CR  = (4U<<25)|DMA_SxCR_DIR_0|DMA_SxCR_MINC|(1U<<16)|DMA_SxCR_TCIE;
    NVIC_SetPriority(USART3_IRQn,       6); NVIC_EnableIRQ(USART3_IRQn);
    NVIC_SetPriority(DMA1_Stream3_IRQn, 6); NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void BSP_UART3_SendDMA(const uint8_t *buf, uint16_t len) {
    while (DMA1_Stream3->CR & DMA_SxCR_EN);
    USART3->SR;
    DMA1_Stream3->M0AR = (uint32_t)buf;
    DMA1_Stream3->NDTR = len;
    DMA1_Stream3->CR  |= DMA_SxCR_EN;
}

void USART3_IRQHandler(void) {
    if (USART3->SR & USART_SR_RXNE)
        BSP_UART3_RxCB((uint8_t)(USART3->DR & 0xFF));
}

void DMA1_Stream3_IRQHandler(void) {
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    }
}
