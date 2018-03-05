/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/08 8:10p $
 * @brief    Demonstrate the BPWM double buffer feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


uint32_t duty30, duty60;
void BPWM0_IRQHandler(void)
{
    static int toggle = 0;  /* First two already fill into BPWM, so start from 30% */

    /* Update BPWM channel 0 duty */
    if(toggle == 0) {
        BPWM->CMPDAT0 = duty30;
    } else {
        BPWM->CMPDAT0 = duty60;
    }
    toggle ^= 1;
    /* Clear channel 0 period interrupt flag */
    BPWM_ClearPeriodIntFlag(BPWM, 0);
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
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_BPWMCKEN_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Set GPC0 multi-function pins for BPWM Channel0 */
    SYS->GPC_MFP = (SYS->GPC_MFP & (~SYS_GPC_MFP_PC0MFP_Msk));
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_BPWM_CH0;
    /* Set GPB2 multi-function pins for BPWM Channel1 */
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB2MFP_Msk));
    SYS->GPB_MFP |= SYS_GPB_MFP_PB2_BPWM_CH1;

    /* Set GPC0, GPB2 as output mode */
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE0_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE2_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* BPWM channel 0 wave form of this sample changed between 30% and 60% duty ratio */
    /* BPWM channel 0 frequency is 1000Hz, duty 30% */
    BPWM->CLKPSC = (BPWM->CLKPSC & ~BPWM_CLKPSC_CLKPSC01_Msk) | 0x01;
    BPWM->CLKDIV = (BPWM->CLKDIV & ~BPWM_CLKDIV_CLKDIV0_Msk) | BPWM_CLK_DIV_1;
    BPWM->CTL |= BPWM_CTL_CNTMODE0_Msk;
    BPWM->CMPDAT0 = 7199;
    BPWM->PERIOD0 = 23999;

    /* Save 30% duty setting */
    duty30 = BPWM->CMPDAT0;
    /* Calculate 60% duty setting. CMPDAT store the actual value minus 1. */
    duty60 = (duty30 + 1) * 2 - 1;

    /* Enable output of all BPWM channel 0 */
    BPWM->POEN |= BPWM_POEN_POEN0_Msk;

    /* Enable BPWM channel 0 period interrupt */
    BPWM->INTEN = (BPWM->INTEN & ~BPWM_INTEN_PINTTYPE_Msk) | BPWM_INTEN_PIEN0_Msk;
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM->CTL |= BPWM_CTL_CNTEN0_Msk;

    /* Fill second duty setting immediately after PWM start */
    BPWM->CMPDAT0 = duty60;

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
