/******************************************************************************
 * @file     i2c_transfer.h
 * @brief    I2C ISP slave header file
 * @version  1.0.0
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __I2C_TRANS_H__
#define __I2C_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bI2cDataReady;
extern volatile uint8_t bI2cSlvEndFlag;
extern uint8_t i2c_rcvbuf[];

#define UI2C_STATUS     (UI2C_PROTSTS_STARIF_Msk | UI2C_PROTSTS_STORIF_Msk | UI2C_PROTSTS_NACKIF_Msk | \
                                            UI2C_PROTSTS_ARBLOIF_Msk | UI2C_PROTSTS_ERRIF_Msk | UI2C_PROTSTS_ACKIF_Msk | UI2C_PROTSTS_SLAREAD_Msk)

/*-------------------------------------------------------------*/
void UI2C_Init(void);
void UI2C_SlaveTRx(UI2C_T *ui2c, uint32_t u32Status);

#endif  /* __I2C_TRANS_H__ */
