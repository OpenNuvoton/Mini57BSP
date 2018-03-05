/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/07 10:59a $
 * @brief    Use pin PB.3 to demonstrates timer event counter function
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void TMR0_IRQHandler(void)
{
    printf("Count 1000 falling events! Test complete\n");
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

    /* Set Timer event counting pin */
    SYS->GPB_MFP = SYS_GPB_MFP_PB3_T0;

    /* Set GPB3 as Input mode */
    PB->MODE = PB->MODE & ~(GPIO_MODE_MODE3_Msk);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    int i;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\nThis sample code use TM0_CNT_OUT(PB.3) to count PB.2 input event\n");
    printf("Please connect PB.3 to PB.2, press any key to continue\n");
    getchar();

    PB->DOUT |= 1 << GPIO_DOUT_DOUT2_Pos;                     /* Set init state to high */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE2_Msk) | (0x1 << GPIO_MODE_MODE2_Pos);  /* Set to output mode */

    /* Reset Timer 0 counter value */
    TIMER0->CTL = 0x0;

    /* Give prescale and compare value to what we need in event counter mode. */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER0->CMP = 1000;
    TIMER0->CTL = TIMER0->CTL | TIMER_ONESHOT_MODE;

    /* Counter increase on falling edge */
    TIMER0->EXTCTL = (TIMER0->EXTCTL & ~TIMER_EXTCTL_CNTPHASE_Msk) | TIMER_COUNTER_FALLING_EDGE;
    TIMER0->CTL |= TIMER_CTL_EXTCNTEN_Msk;

    /* Start Timer 0 and enable timer interrupt */
    TIMER0->CTL |= (TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk);
    NVIC_EnableIRQ(TMR0_IRQn);

    for(i = 0; i < 1000; i++) {
        PB->DOUT &= ~(1 << GPIO_DOUT_DOUT2_Pos); /* low */
        PB->DOUT |= (1 << GPIO_DOUT_DOUT2_Pos);  /* high */
    }

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
