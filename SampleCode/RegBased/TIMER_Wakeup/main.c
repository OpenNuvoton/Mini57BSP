/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/07 2:05p $
 * @brief    Use timer to wake up system from Power-down mode periodically.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void TMR0_IRQHandler(void)
{
    /* Clear wake up flag */
    TIMER_ClearWakeupFlag(TIMER0);
    /* Clear interrupt flag */
    TIMER_ClearIntFlag(TIMER0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable 48MHz HIRC */
    CLK->PWRCTL = CLK->PWRCTL | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for 48MHz clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* HCLK Clock source from HIRC */
    CLK->CLKSEL0 = CLK->CLKSEL0 | CLK_HCLK_SRC_HIRC;

    /* Enable IP clock */
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_USCI0CKEN_Msk;
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_TMR0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_TMR0_SRC_LIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

void UUART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS->IPRST1 |= SYS_IPRST1_USCI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_USCI0RST_Msk;

    /* Configure USCI0 as UART mode */
    UUART0->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Set UART function mode */
    UUART0->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART0->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART0->BRGEN = (51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos); /* Set UART baud rate as 115200bps */
    UUART0->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */
}

int main()
{
    int i = 0;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    /*Initial Timer0 to periodic mode with 1Hz */
    TIMER0->CMP = __LIRC;
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER0->CTL = TIMER0->CTL | TIMER_PERIODIC_MODE;

    /* Enable timer wake up system */
    TIMER0->CTL |= TIMER_CTL_WKEN_Msk;
    /* Enable Timer0 interrupt */
    TIMER0->CTL |= TIMER_CTL_INTEN_Msk;
    NVIC_EnableIRQ(TMR0_IRQn);
    /* Start Timer0 counting */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;
    /* Unlock protected registers */
    SYS_UnlockReg();
    while(1)
    {
        /* Enable PD.0 (nRESET pin) interrupt that trigger by falling edge to make sure
           RESET button can wake up system from power down mode. */
        PD->INTTYPE &= (~BIT0);     /* edge trigger for PD0 */
        PD->INTEN   |=   BIT0;      /* enable falling or low trigger for PD0 */

        /* enable M0 register SCR[SEVONPEND] and SCR[SLEEPDEEP] */
        SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SEVONPEND_Msk);
        /* clear interrupt status and enable wake up interrupt */
        CLK->PWRCTL |= (CLK_PWRCTL_PDWKIF_Msk | CLK_PWRCTL_PDWKIEN_Msk);
        /* enable system power-down feature */
        CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);
        /* execute Wait For Interrupt; Enter power-down mode since CLK_PWRCTL_PDEN_Msk is 1 */
        __WFI();

        printf("Wake %d\n", i++);

        /* Wait USCI UART buffer empty to get a cleaner console out */
        UUART0->PROTSTS = (UUART_PROTSTS_TXENDIF_Msk | UUART_PROTSTS_TXSTIF_Msk);
        while(!((UUART0->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk) >> UUART_BUFSTS_TXEMPTY_Pos));
        if(UUART0->PROTSTS & UUART_PROTSTS_TXSTIF_Msk)
            while((UUART0->PROTSTS & UUART_PROTSTS_TXENDIF_Msk) != UUART_PROTSTS_TXENDIF_Msk);
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
