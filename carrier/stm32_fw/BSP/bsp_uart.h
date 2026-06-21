#pragma once
#include <stdint.h>
/* USART3 921600bps  PB10=TX PB11=RX */
void BSP_UART3_Init(void);
void BSP_UART3_SendDMA(const uint8_t *buf, uint16_t len);
void BSP_UART3_RxCB(uint8_t byte);
