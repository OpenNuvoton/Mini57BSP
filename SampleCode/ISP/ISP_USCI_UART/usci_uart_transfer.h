/******************************************************************************
 * @file     usci_uart_transfer.h
 * @brief    USCI UART ISP slave header file
 * @version  3.0.0
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE            64

/*-------------------------------------------------------------*/

extern uint8_t  uart_rcvbuf[];
extern uint8_t volatile bUartDataReady;
extern uint8_t volatile bufhead;
extern uint32_t volatile rcvsize;

/*-------------------------------------------------------------*/
void USCI0_Init(void);
void USCI0_IRQHandler(void);
uint32_t PutString(void);

#endif  /* __UART_TRANS_H__ */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
