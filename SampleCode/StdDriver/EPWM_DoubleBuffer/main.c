/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Demonstrate the EPWM double buffer feature.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


uint32_t duty30, duty60;
void EPWM_IRQHandler(void)
{
    static int toggle = 0;  /* First two already fill into EPWM, so start from 30% */

    /* Update EPWM channel 0 duty */
    if(toggle == 0)
    {
        EPWM_SET_CMR(EPWM, 0, duty30);
    }
    else
    {
        EPWM_SET_CMR(EPWM, 0, duty60);
    }
    toggle ^= 1;
    /* Clear channel 0 period interrupt flag */
    EPWM_ClearPeriodIntFlag(EPWM, 0);
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
    CLK_EnableModuleClock(EPWM_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Set GPA multi-function pins for EPWM Channel0 */
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk)) | SYS_GPA_MFP_PA0_EPWM_CH0;

    /* Set GPA0 as output mode */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* EPWM channel 0 wave form of this sample changed between 30% and 60% duty ratio */
    EPWM_ConfigOutputChannel(EPWM, 0, 1000, 30);

    /* Save 30% duty setting */
    duty30 = EPWM->CMPDAT[0];
    /* Calculate 60% duty setting. */
    duty60 = duty30 * 2;

    /* Enable output of all PWM channel 0 */
    EPWM_EnableOutput(EPWM, 1);

    /* Enable EPWM channel 0 period interrupt */
    EPWM_EnablePeriodInt(EPWM, 0, 0);
    NVIC_EnableIRQ(EPWM_IRQn);

    /* Start */
    EPWM_Start(EPWM, 0x1);

    /* Fill second duty setting immediately after EPWM start */
    EPWM_SET_CMR(EPWM, 0, duty60);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
