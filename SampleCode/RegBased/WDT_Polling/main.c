/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/07 3:42p $
 * @brief    Use polling mode to check WDT time-out state and reset WDT after
 *           time out occurs
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable 48MHz HIRC */
    CLK->PWRCTL = CLK->PWRCTL | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for 48MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* HCLK Clock source from HIRC */
    CLK->CLKSEL0 = CLK->CLKSEL0 | CLK_HCLK_SRC_HIRC;

    /* Enable IP clock */
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_WDTCKEN_Msk);

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK->CLKSEL1 | CLK_WDT_SRC_LIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\nThis sample code demonstrate using WDT in polling mode\n");

    /* WDT register is locked, so it is necessary to unlock protect register before configure WDT */
    SYS_UnlockReg();

    /* WDT timeout every 2^14 WDT clock, disable system reset, disable wake up system */
    WDT->CTL = WDT_TIMEOUT_2POW14 | WDT_CTL_WDTEN_Msk | (DISABLE << WDT_CTL_RSTEN_Pos) | (DISABLE << WDT_CTL_WKEN_Pos);

    /* Enable WDT timeout interrupt */
    WDT->CTL |= WDT_CTL_INTEN_Msk;

    while(1) {
        /* WDT timeout flag set */
        if(WDT_GET_TIMEOUT_INT_FLAG()) {
            /* Reset WDT and clear time out flag */
            WDT_CLEAR_TIMEOUT_INT_FLAG();
            printf("Reset WDT counter\n");
        }
    }

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
