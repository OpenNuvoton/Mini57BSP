/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
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

    if(cnt == 0) {
        t0 = TIMER_GetCaptureData(TIMER0);
        cnt++;
    } else if(cnt == 1) {
        t1 = TIMER_GetCaptureData(TIMER0);
        cnt++;
        if(t0 > t1) {
            /* over run, drop this data and do nothing */
        } else {
            printf("Input frequency is %dHz\n", 1000000 / (t1 - t0));
        }
    } else {
        cnt = 0;
    }

    TIMER_ClearCaptureIntFlag(TIMER0);

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable 48MHz HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for 48MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* HCLK Clock source from HIRC */
    CLK_SetHCLK(CLK_HCLK_SRC_HIRC,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(ACMP_MODULE);
	
    /* Select IP clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMR0_SRC_HIRC, 0);	
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Set GPB0 multi-function pin for ACMP0 positive input pin */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_ACMP0_P0;

    /* Set GPB0 as input mode */
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);

    /* Disable digital input path of analog pin ACMP0_P to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1 << 0));	

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\nThis sample code demonstrate timer free counting mode.\n");
    printf("Please connect input source with ACMP0 positive input pin (PB.0), press any key to continue\n");
    getchar();

    /* Give a dummy target frequency here. Will over write capture resolution with macro */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    /* Update prescale to set proper resolution. */
    /* Timer 0 clock source is 48MHz, to set resolution to 1us, we need to */
    /* set clock divider to 48. e.g. set prescale to 48 - 1 = 47 */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 47);

    /* Set compare value as large as possible, so don't need to worry about counter overrun too frequently. */
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    /* Configure Timer 0 free counting mode, capture TDR value on rising edge */
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_RISING_EDGE);

    /* Configure ACMP0. Enable ACMP0 and select internal reference voltage as negative input. */
    ACMP_Open(ACMP, 0, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Set ACMP0 rising edge as interrupt condition. */
    ACMP_SELECT_INT_COND(ACMP, 0, ACMP_CTL_INTPOL_R);

    /* Start Timer 0 */
    TIMER_Start(TIMER0);

    /* Enable timer interrupt */
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
