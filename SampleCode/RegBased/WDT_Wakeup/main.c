/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/07 4:33p $
 * @brief    Use WDT to wake system up from power-down mode periodically
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void WDT_IRQHandler(void)
{

    /* Clear WDT interrupt flag */
    WDT_CLEAR_TIMEOUT_INT_FLAG();

    /* Check WDT wake up flag */
    if(WDT_GET_TIMEOUT_WAKEUP_FLAG()) {
        printf("Wake up by WDT\n");
        /* Clear WDT wake up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

}

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

    printf("\nThis sample code demonstrate using WDT to wake system up from power down mode\n");

    /* WDT register is locked, so it is necessary to unlock protect register before configure WDT */
    SYS_UnlockReg();

    /* WDT timeout every 2^14 WDT clock, disable system reset, enable wake up system */
    WDT->CTL = WDT_TIMEOUT_2POW14 | WDT_CTL_WDTEN_Msk | (DISABLE << WDT_CTL_RSTEN_Pos) | (ENABLE << WDT_CTL_WKEN_Pos);

    /* Enable WDT timeout interrupt */
    WDT->CTL |= WDT_CTL_INTEN_Msk;
    NVIC_EnableIRQ(WDT_IRQn);

    while(1) {
        /* Wait USCI UART buffer empty to get a cleaner console out */
        UUART0->PROTSTS = (UUART_PROTSTS_TXENDIF_Msk | UUART_PROTSTS_TXSTIF_Msk);
        while(!((UUART0->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk) >> UUART_BUFSTS_TXEMPTY_Pos));
        if(UUART0->PROTSTS & UUART_PROTSTS_TXSTIF_Msk)
            while((UUART0->PROTSTS & UUART_PROTSTS_TXENDIF_Msk) != UUART_PROTSTS_TXENDIF_Msk);
        CLK_PowerDown();
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
