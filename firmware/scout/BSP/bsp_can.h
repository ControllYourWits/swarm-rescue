#pragma once
#include <stdint.h>
/* CAN1  PA11=RX AF9  PA12=TX AF9  1Mbps APB1=42MHz */
void BSP_CAN1_Init(void);
void BSP_CAN1_Send(uint32_t id, const uint8_t *data, uint8_t len);
/* 接收回调，由用户实现 */
void BSP_CAN1_RxCB(uint32_t id, const uint8_t *data, uint8_t len);
