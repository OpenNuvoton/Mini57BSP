/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/07 3:07p $
 * @brief    Use the ACMP0 positive input pin to demonstrate timer free counting mode
 *           function. And displays the measured input frequency to console
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void TMR0_IRQHandler(void)
{
    /* printf takes long time and affect the freq. calculation, we only print out once a while */
    static int cnt = 0;
    static uint32_t t0, t1;

    if(cnt == 0)
    {
        t0 = TIMER_GetCaptureData(TIMER0);
        cnt++;
    }
    else if(cnt == 1)
    {
        t1 = TIMER_GetCaptureData(TIMER0);
        cnt++;
        if(t0 > t1)
        {
            /* over run, drop this data and do nothing */
        }
        else
        {
            printf("Input frequency is %dHz\n", 1000000 / (t1 - t0));
        }
    }
    else
    {
        cnt = 0;
    }

    TIMER_ClearCaptureIntFlag(TIMER0);

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
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_TMR0CKEN_Msk | CLK_APBCLK_ACMPCKEN_Msk);

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK->CLKSEL1 | CLK_TMR0_SRC_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Set GPB0 multi-function pin for ACMP0 positive input pin */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~SYS_GPB_MFP_PB0MFP_Msk) | SYS_GPB_MFP_PB0_ACMP0_P0;

    /* Set GPB0 as input mode */
    PB->MODE = PB->MODE & ~(GPIO_MODE_MODE0_Msk);

    /* Disable digital input path of analog pin ACMP0_P to prevent leakage */
    PB->DINOFF |= GPIO_DINOFF_DINOFF0_Msk;

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
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\nThis sample code demonstrate timer free counting mode.\n");
    printf("Please connect input source with ACMP0 positive input pin (PB.0), press any key to continue\n");
    getchar();

    /* Update prescale to set proper resolution. */
    /* Timer 0 clock source is 48MHz, to set resolution to 1us, we need to */
    /* set clock divider to 48. e.g. set prescale to 48 - 1 = 47 */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 47);

    /* Set compare value as large as possible, so don't need to worry about counter overrun too frequently. */
    TIMER0->CMP = 0xFFFFFF;

    /* Configure Timer 0 free counting mode, capture TDR value on rising edge */
    TIMER0->EXTCTL = (TIMER0->EXTCTL & ~(TIMER_EXTCTL_CAPFUNCS_Msk | TIMER_EXTCTL_CAPEDGE_Msk)) |
                     TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_RISING_EDGE | TIMER_EXTCTL_CAPEN_Msk;

    /* Configure ACMP0. Enable ACMP0 and select internal reference voltage as negative input. */
    ACMP->CTL[0] = (ACMP->CTL[0] & ~(ACMP_CTL_CPNSEL_Msk | ACMP_CTL_ACMPHYSEN_Msk)) |
                   (ACMP_CTL_NEGSEL_VBG | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_ACMPEN_Msk);

    /* Set ACMP0 rising edge as interrupt condition. */
    ACMP->CTL[0] = (ACMP->CTL[0] & ~ACMP_CTL_EDGESEL_Msk) | ACMP_CTL_INTPOL_R;

    /* Start Timer 0 */
    TIMER0->CTL |= (TIMER_CTL_CNTEN_Msk | TIMER_PERIODIC_MODE);

    /* Enable timer interrupt */
    TIMER0->EXTCTL |= TIMER_EXTCTL_CAPIEN_Msk;
    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
