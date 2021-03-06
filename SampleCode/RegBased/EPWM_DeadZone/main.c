/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/09 10:50a $
 * @brief    Demonstrate the EPWM dead-zone feature.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void EPWM_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
    if(++cnt == 100)
    {
        if(out)
            PA->MODE |= 0x555;
        else
            PA->MODE &= ~0x555;
        out ^= 1;
        cnt = 0;
    }
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
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* HCLK Clock source from HIRC */
    CLK->CLKSEL0 = CLK->CLKSEL0 | CLK_HCLK_SRC_HIRC;

    /* Enable IP clock */
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_EPWMCKEN_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Set GPA multi-function pins for EPWM Channel0~5 */
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk)) | SYS_GPA_MFP_PA0_EPWM_CH0;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA1MFP_Msk)) | SYS_GPA_MFP_PA1_EPWM_CH1;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA2MFP_Msk)) | SYS_GPA_MFP_PA2_EPWM_CH2;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA3MFP_Msk)) | SYS_GPA_MFP_PA3_EPWM_CH3;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA4MFP_Msk)) | SYS_GPA_MFP_PA4_EPWM_CH4;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA5MFP_Msk)) | SYS_GPA_MFP_PA5_EPWM_CH5;

    /* Set GPA0~5 as output mode */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE1_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE1_Pos);
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE3_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE3_Pos);
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE4_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE4_Pos);
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE5_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

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

    printf("\nThis sample code will output EPWM channel to with different\n");
    printf("duty, enable dead zone function of all EPWM pairs.\n");
    printf("And also enable/disable EPWM output every 1 second.\n");
    /* EPWM0 frequency is 100Hz, duty 30% */
    EPWM->CLKDIV = EPWM_CLK_DIV_8;
    EPWM->CTL = (EPWM->CTL & ~EPWM_CTL_CNTTYPE_Msk) | (EPWM_CTL_CNTMODE_Msk);
    EPWM->CMPDAT[0] = 18000;
    EPWM->PERIOD = 59999;
    /* set duration */
    EPWM->DTCTL = (EPWM->DTCTL & ~(EPWM_DTCTL_DTCNT01_Msk)) | 150;
    /* enable dead zone */
    EPWM->CTL |= EPWM_CTL_DTCNT01_Msk;

    /* EPWM2 frequency is 100Hz, duty 50% */
    EPWM->CMPDAT[2] = 30000;
    /* set duration */
    EPWM->DTCTL = (EPWM->DTCTL & ~(EPWM_DTCTL_DTCNT23_Msk)) | (200 << EPWM_DTCTL_DTCNT23_Pos);
    /* enable dead zone */
    EPWM->CTL |= EPWM_CTL_DTCNT23_Msk;

    /* EPWM4 frequency is 100Hz, duty 70% */
    EPWM->CMPDAT[4] = 42000;
    /* set duration */
    EPWM->DTCTL = (EPWM->DTCTL & ~(EPWM_DTCTL_DTCNT45_Msk)) | (100 << EPWM_DTCTL_DTCNT45_Pos);
    /* enable dead zone */
    EPWM->CTL |= EPWM_CTL_DTCNT45_Msk;

    /* Enable complementary mode */
    EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM);

    /* Enable output of all EPWM channels */
    PA->MODE |= 0x555;

    /* Enable EPWM channel 0 period interrupt, use channel 0 to measure time. */
    EPWM->INTEN |= (EPWM_INTEN_PIEN_Msk);
    NVIC_EnableIRQ(EPWM_IRQn);

    /* Start */
    EPWM->CTL |= 0x3F;

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
