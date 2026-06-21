#pragma once
#include <stdint.h>
/* USART3 921600bps DMA发送 中断接收
 * PB10=TX AF7  PB11=RX AF7  APB1=42MHz BRR=0x02D9 */
void BSP_UART3_Init(void);
void BSP_UART3_SendDMA(const uint8_t *buf, uint16_t len);
/* 单字节接收回调，由用户实现 */
void BSP_UART3_RxCB(uint8_t byte);
