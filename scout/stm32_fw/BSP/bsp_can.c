/**
 * bsp_can.c — CAN1寄存器驱动 1Mbps
 * APB1=42MHz BTR: BRP=2(+1=3), TS1=9, TS2=3
 */
#include "bsp_can.h"
#include "stm32f4xx.h"

void BSP_CAN1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    /* PA11 PA12 AF9 */
    GPIOA->MODER  &= ~((3U<<22)|(3U<<24));
    GPIOA->MODER  |=  ((2U<<22)|(2U<<24));
    GPIOA->OSPEEDR|=  ((3U<<22)|(3U<<24));
    GPIOA->AFR[1] &= ~((0xFU<<12)|(0xFU<<16));
    GPIOA->AFR[1] |=  ((9U<<12) |(9U<<16));
    /* init mode */
    CAN1->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK));
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    /* 1Mbps */
    CAN1->BTR = (3U<<20)|(9U<<16)|(2U);
    /* filter pass-all */
    CAN1->FMR  |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~(1U<<0);
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FM1R &= ~(1U<<0); CAN1->FS1R |= (1U<<0);
    CAN1->FFA1R &= ~(1U<<0); CAN1->FA1R |= (1U<<0);
    CAN1->FMR  &= ~CAN_FMR_FINIT;
    CAN1->IER  |= CAN_IER_FMPIE0;
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK);
}

void BSP_CAN1_Send(uint32_t id, const uint8_t *d, uint8_t len) {
    uint32_t t = 1000;
    while (!(CAN1->TSR & CAN_TSR_TME0) && --t);
    if (!t) return;
    CAN1->sTxMailBox[0].TIR  = (id << 21U);
    CAN1->sTxMailBox[0].TDTR = len & 0x0FU;
    CAN1->sTxMailBox[0].TDLR = (uint32_t)d[0]|((uint32_t)d[1]<<8)|
                                ((uint32_t)d[2]<<16)|((uint32_t)d[3]<<24);
    CAN1->sTxMailBox[0].TDHR = (uint32_t)d[4]|((uint32_t)d[5]<<8)|
                                ((uint32_t)d[6]<<16)|((uint32_t)d[7]<<24);
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

void CAN1_RX0_IRQHandler(void) {
    uint32_t id  = (CAN1->sFIFOMailBox[0].RIR >> 21U) & 0x7FFU;
    uint8_t  dlc = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR & 0x0FU);
    uint8_t  d[8];
    uint32_t lo = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t hi = CAN1->sFIFOMailBox[0].RDHR;
    for (int i=0;i<4;i++) { d[i]=  (uint8_t)(lo>>(i*8));
                             d[i+4]=(uint8_t)(hi>>(i*8)); }
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    BSP_CAN1_RxCB(id, d, dlc);
}
