/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for EADC Timer Trigger feature.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_HCLK_SRC_HIRC;
    CLK->CLKDIV  = (CLK->CLKDIV  & ~CLK_CLKDIV_HCLKDIV_Msk)  | CLK_CLKDIV_HCLK(1);

    /* Enable USCI0 IP clock */
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_USCI0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) |
                   (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);
    PD->MODE = (PB->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) |
               (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos) | (GPIO_MODE_INPUT << GPIO_MODE_MODE6_Pos);

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


void EADC0_IRQHandler(void)
{
    if(EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk))
    {
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk); /* clear EADC0 interrupt flag */
        printf("Get EADC0 FIFO 0 data = %lu\n", EADC_GET_CONV_DATA(EADC, EADC_EADC0_DAT0));
    }
}


void TMR0_IRQHandler(void)
{
    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER0);
    printf("Timer0 expired !!\n");
}


int main()
{
    char ch;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will configure Timer0 to trigger EADC0 channel 0 periodically
     * and print conversion results.
     */
    printf("+-------------------------------------------+\n");
    printf("| Mini57 EADC for Timer Trigger Sample Code |\n");
    printf("+-------------------------------------------+\n");

    /* Enable EADC clock */
    CLK->APBCLK |= CLK_APBCLK_ADCCKEN_Msk;

    /* Set EADC clock source to HIRC */
    CLK->CLKDIV  = (CLK->CLKDIV  & ~CLK_CLKDIV_ADCDIV_Msk)  | CLK_CLKDIV_EADC(4);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_ADCSEL_Msk) | CLK_EADC_SRC_HIRC;

    /* Reset IP */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    printf("Set GPB0 pin as EADC0 AIN0 input pin.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB0MFP_Msk)) | SYS_GPB_MFP_PB0_ADC0_CH0;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE0_Pos);
    PB->DINOFF |= GPIO_DINOFF_DINOFF0_Msk;

    /*--- Initial Timer to trigger ADC */

    /* Enable TIMER0 clock */
    CLK->APBCLK |= CLK_APBCLK_TMR0CKEN_Msk;

    printf("Set Timer0 to periodic 1Hz. It will trigger EADC 1 times per second.\n");
    TIMER0->CTL = TIMER_PERIODIC_MODE | 3;
    TIMER0->CMP = 12000000;

    TIMER_EnableInt(TIMER0);

    NVIC_EnableIRQ(TMR0_IRQn);

    /*--- Initial EADC */

    /* Enable EADC */
    /* Configure EADC0: channel 0 (AIN0), Timer0 trigger */
    /* Configure EADC conversion mode to Independent Simple Mode */
    EADC->CTL = (EADC->CTL & ~(EADC_CTL_ADC0CHSEL_Msk | EADC_CTL_ADC0HWTRGEN_Msk | EADC_CTL_ADCMODE_Msk)) |
                ((EADC_EADC0_0 << EADC_CTL_ADC0CHSEL_Pos) | (0x0 << EADC_CTL_ADCMODE_Pos) | EADC_CTL_ADCEN_Msk | EADC_CTL_ADC0HWTRGEN_Msk);
    EADC->TRGSOR = (EADC->TRGSOR & ~(EADC_TRGSOR_ADC0TRGSOR_Msk | EADC_TRGSOR_ADC0STADCSEL_Msk | EADC_TRGSOR_ADC0PWMTRGSEL_Msk))
                   | (EADC_TIMER0_TRIGGER << EADC_TRGSOR_ADC0TRGSOR_Pos);

    /* Configure EADC sample time to 6 EADC clocks */
    EADC->SMPCNT = (EADC->SMPCNT & ~(EADC_SMPCNT_ADCSMPCNT_Msk)) |
                   (5 << EADC_SMPCNT_ADCSMPCNT_Pos);

    EADC_ENABLE_INT(EADC, EADC_CTL_ADC0IEN_Msk);

    NVIC_EnableIRQ(EADC0_IRQn);

    /* Begin to do ADC conversion. */
    ch = 0;

    TIMER_Start(TIMER0);

    while (ch != 'q')
    {
        printf("Press 'q' to quit.\n");
        ch = getchar();
    }

    NVIC_DisableIRQ(TMR0_IRQn);

    /* Close TIMER0 */
    TIMER0->CTL = 0;
    TIMER0->EXTCTL = 0;

    CLK->APBCLK &= ~CLK_APBCLK_TMR0CKEN_Msk;

    NVIC_DisableIRQ(EADC0_IRQn);

    /* Close EADC */
    EADC->CTL = (EADC->CTL & ~(EADC_CTL_ADCEN_Msk)) |
                ( (EADC_EADC0_7 << EADC_CTL_ADC0CHSEL_Pos) |                    /* Switching to channel Vss to save power */
                  ((EADC_EADC1_7-EADC_EADC1_0) << EADC_CTL_ADC1CHSEL_Pos) );    /* Switching to channel Vss to save power */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    /* Disable EADC clock */
    CLK->APBCLK &= ~CLK_APBCLK_ADCCKEN_Msk;

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
