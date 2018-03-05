/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/08 1:12p $
 * @brief    Demonstrate the BPWM dead-zone feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void BPWM0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
    if(++cnt == 100) {
        if(out) {
            BPWM->POEN |= 0x3;
            PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE0_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
            PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE2_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);
        } else {
            BPWM->POEN &= ~0x3;
            PC->MODE &= ~(GPIO_MODE_MODE0_Msk);
            PB->MODE &= ~(GPIO_MODE_MODE2_Msk);
        }
        out ^= 1;
        cnt = 0;
    }
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

    printf("\nThis sample code will output BPWM channel 0 to with different\n");
    printf("frequency and duty, enable dead zone function of all BPWM pairs.\n");
    printf("And also enable/disable BPWM output every 1 second.\n");
    /* BPWM0 frequency is 100Hz, duty 30% */
    BPWM->CLKPSC = (BPWM->CLKPSC & ~BPWM_CLKPSC_CLKPSC01_Msk) | 0x07;
    BPWM->CLKDIV = (BPWM->CLKDIV & ~BPWM_CLKDIV_CLKDIV0_Msk) | BPWM_CLK_DIV_1;
    BPWM->CTL |= BPWM_CTL_CNTMODE0_Msk;
    BPWM->CMPDAT0 = 17999;
    BPWM->PERIOD0 = 59999;

    BPWM->CLKPSC = (BPWM->CLKPSC & ~BPWM_CLKPSC_DTI01_Msk) | (100 << BPWM_CLKPSC_DTI01_Pos);
    BPWM->CTL |= BPWM_CTL_DTCNT01_Msk;

    /* Enable output of all BPWM channels */
    BPWM->POEN |= 0x3;

    /* Enable BPWM channel 0 period interrupt, use channel 0 to measure time. */
    BPWM->INTEN = (BPWM->INTEN & ~BPWM_INTEN_PINTTYPE_Msk) | BPWM_INTEN_PIEN0_Msk;
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM->CTL |= (BPWM_CTL_CNTEN0_Msk | BPWM_CTL_CNTEN1_Msk);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
