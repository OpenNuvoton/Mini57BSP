/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/09 11:19a $
 * @brief    Demonstrate the EPWM double buffer feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


uint32_t duty30, duty60;
void EPWM_IRQHandler(void)
{
    static int toggle = 0;  /* First two already fill into EPWM, so start from 30% */

    /* Update EPWM channel 0 duty */
    if(toggle == 0) {
        EPWM->CMPDAT[0]= duty30;
    } else {
        EPWM->CMPDAT[0]= duty60;
    }
    toggle ^= 1;
    /* Clear channel 0 period interrupt flag */
    EPWM->INTSTS = EPWM_INTSTS_PIF_Msk;
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
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_EPWMCKEN_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Set GPA multi-function pins for EPWM Channel0 */
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk));
    SYS->GPA_MFP |= SYS_GPA_MFP_PA0_EPWM_CH0;

    /* Set GPA0 as output mode */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* EPWM channel 0 wave form of this sample changed between 30% and 60% duty ratio */
    EPWM->CTL = (EPWM->CTL & ~EPWM_CTL_CNTTYPE_Msk) | (EPWM_CTL_CNTMODE_Msk);
    EPWM->CMPDAT[0] = 14400;
    EPWM->PERIOD = 47999;

    /* Save 30% duty setting */
    duty30 = EPWM->CMPDAT[0];
    /* Calculate 60% duty setting. */
    duty60 = duty30 * 2;

    /* Enable output of all PWM channel 0 */
    EPWM_EnableOutput(EPWM, 1);

    /* Enable EPWM channel 0 period interrupt */
    EPWM->INTEN |= (EPWM_INTEN_PIEN_Msk);
    NVIC_EnableIRQ(EPWM_IRQn);

    /* Start */
    EPWM->CTL |= EPWM_CTL_CNTEN0_Msk;

    /* Fill second duty setting immediately after EPWM start */
    EPWM->CMPDAT[0]= duty60;

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
