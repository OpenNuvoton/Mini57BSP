/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/07 11:27a $
 * @brief    Use the timer periodic mode to generate timer interrupt every 1 second
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void TMR0_IRQHandler(void)
{
    static uint32_t sec = 1;
    printf("%d sec\n", sec++);

    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER0);

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
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_USCI0CKEN_Msk;
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_TMR0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK->CLKSEL1 | CLK_TMR0_SRC_HIRC;

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

    printf("\nThis sample code use timer to generate interrupt every 1 second \n");

    /* Set timer frequency to 1HZ */
    TIMER0->CMP = (__HIRC/4);
    TIMER_SET_PRESCALE_VALUE(TIMER0, 3);
    TIMER0->CTL = TIMER0->CTL | TIMER_PERIODIC_MODE;

    /* Enable timer interrupt */
    TIMER0->CTL = TIMER0->CTL | TIMER_CTL_INTEN_Msk;
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer 0 */
    TIMER0->CTL = TIMER0->CTL | TIMER_CTL_CNTEN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
