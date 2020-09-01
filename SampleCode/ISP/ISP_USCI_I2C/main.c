/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x33
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "usci_i2c_transfer.h"
#include "isp_user.h"

void SYS_Init(void)
{
    /* Enable Internal and External RC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_HCLK_SRC_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLKDIV_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);
    SystemCoreClock = __HIRC / 1;               // HCLK
    CyclesPerUs     = __HIRC / 1000000;         // For SYS_SysTickDelay()
    /* Enable USCI module clock */
    CLK->APBCLK |= CLK_APBCLK_USCI0CKEN_Msk;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPA multi-function pins for USCI I2C0 GPA3(SCL) and GPA2(SDA) */
    SYS->GPA_MFP = (SYS->GPA_MFP & ~(SYS_GPA_MFP_PA3MFP_Msk | SYS_GPA_MFP_PA2MFP_Msk)) | (SYS_GPA_MFP_PA3_I2C0_SCL | SYS_GPA_MFP_PA2_I2C0_SDA);
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/


int main(void)
{
    uint32_t cmd_buff[16];
    uint32_t u32Status;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init USCI I2C */
    UI2C_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = 0x7600;  // 29.5 K
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    while (1)
    {
        u32Status = UI2C_GET_PROT_STATUS(UI2C0) & UI2C_STATUS;

        if(u32Status != 0)
        {
            UI2C_SlaveTRx(UI2C0, u32Status);
        }

        if ((bI2cDataReady == 1) && (bI2cSlvEndFlag == 1))
        {
            goto _ISP;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {

        if ((bI2cDataReady == 1) && (bI2cSlvEndFlag == 1))
        {
            memcpy(cmd_buff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            bI2cSlvEndFlag = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
            bISPDataReady = 1;
        }

        u32Status = UI2C_GET_PROT_STATUS(UI2C0) & UI2C_STATUS;

        if(u32Status != 0)
        {
            UI2C_SlaveTRx(UI2C0, u32Status);
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
