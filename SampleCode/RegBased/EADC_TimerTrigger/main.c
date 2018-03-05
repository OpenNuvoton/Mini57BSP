/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for EADC Timer Trigger feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


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

    /* Enable USCI0 IP clock */
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_USCI0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Enable IP clock */
    /* CLK_EnableModuleClock(EADC_MODULE); */
    CLK->APBCLK |= CLK_APBCLK_ADCCKEN_Msk;

    /* SYS_ResetModule(EADC_RST); */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    /* CLK_EnableModuleClock(TMR0_MODULE); */
    CLK->APBCLK |= CLK_APBCLK_TMR0CKEN_Msk;

    /* Select IP clock source */

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Lock protected registers */
    SYS_LockReg();
}


void EADC0_IRQHandler(void)
{
    /* if(EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk)) */
    if(EADC->STATUS & EADC_STATUS_ADC0F_Msk)
    {
        /* EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk); */   /* clear EADC0 interrupt flag */
        EADC->STATUS = EADC_STATUS_ADC0F_Msk;

        /* printf("Get EADC0 FIFO 0 data = %d\n", EADC_GET_CONV_DATA(EADC, EADC_EADC0_DAT0)); */
        printf("Get EADC0 FIFO 0 data = %d\n",
            (EADC->DAT[0] & EADC_DAT0_ADC0DAT0_Msk) >> EADC_DAT0_ADC0DAT0_Pos);
    }
}


void TMR0_IRQHandler(void)
{
    /* clear timer interrupt flag */
    /*TIMER_ClearIntFlag(TIMER0); */
    TIMER0->INTSTS = TIMER_INTSTS_TIF_Msk;

    printf("Timer0 expired !!\n");
}


int main()
{
    char ch;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* printf("\n\nPDID 0x%08X\n", SYS_ReadPDID()); */    /* Display PDID */
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
    /* CLK_EnableModuleClock(EADC_MODULE); */
    CLK->APBCLK |= CLK_APBCLK_ADCCKEN_Msk;

    /* Set EADC clock source to HIRC */
    /* CLK_SetModuleClock(EADC_MODULE, CLK_EADC_SRC_HIRC, CLK_CLKDIV_EADC(4)); */
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_ADCDIV_Msk) | (((4) - 1) << CLK_CLKDIV_ADCDIV_Pos);
    CLK->CLKSEL1 &= (~CLK_CLKSEL1_ADCSEL_Msk);
    CLK->CLKSEL1 |= CLK_EADC_SRC_HIRC;

    /* Reset IP */
    /* SYS_ResetModule(EADC_RST); */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    printf("Set GPB0 pin as ADC0 AIN0 input pin.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB0MFP_Msk)) | SYS_GPB_MFP_PB0_ADC0_CH0;

    /* GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT); */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE0_Pos);

    /* GPIO_DISABLE_DIGITAL_PATH(PB, BIT0); */
    PB->DINOFF |= GPIO_DINOFF_DINOFF0_Msk;

    /*--- Initial Timer to trigger ADC */
    printf("Set Timer0 to periodic 1Hz. It will trigger ADC 1 times per second.\n");
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);

    /* TIMER_EnableInt(TIMER0); */
    TIMER0->CTL |= TIMER_CTL_INTEN_Msk;

    NVIC_EnableIRQ(TMR0_IRQn);

    /*--- Initial EADC */
    /* Enable EADC */
    /* EADC_Open(EADC, NULL); */
    EADC->CTL |= EADC_CTL_ADCEN_Msk;

    /* Configure EADC0: channel 0 (AIN0), Timer0 trigger */
    /* EADC_ConfigSampleModule(EADC, EADC_EADC0_0, EADC_TIMER0_TRIGGER, NULL); */
    EADC->CTL = (EADC->CTL & ~EADC_CTL_ADC0CHSEL_Msk) | (EADC_EADC0_0 << EADC_CTL_ADC0CHSEL_Pos);
    EADC->TRGSOR = (EADC->TRGSOR & ~(EADC_TRGSOR_ADC0TRGSOR_Msk | EADC_TRGSOR_ADC0STADCSEL_Msk | EADC_TRGSOR_ADC0PWMTRGSEL_Msk))
                 | (EADC_TIMER0_TRIGGER << EADC_TRGSOR_ADC0TRGSOR_Pos);
    EADC->CTL |= EADC_CTL_ADC0HWTRGEN_Msk;

    /* Configure EADC conversion mode to Independent Simple Mode */
    /* EADC_SET_INDEPENDENT_SIMPLE_MODE(EADC); */
    EADC->CTL = (EADC->CTL & ~EADC_CTL_ADCMODE_Msk) | (0x0 << EADC_CTL_ADCMODE_Pos);

    /* Configure EADC sample time to 6 EADC clocks */
    /* EADC_SetExtendSampleTime(EADC, NULL, 5); */
    EADC->SMPCNT = (EADC->SMPCNT & (~EADC_SMPCNT_ADCSMPCNT_Msk)) | ((5 & 0xF) << EADC_SMPCNT_ADCSMPCNT_Pos);

    /* EADC_ENABLE_INT(EADC, EADC_CTL_ADC0IEN_Msk); */
    EADC->CTL |= EADC_CTL_ADC0IEN_Msk;

    NVIC_EnableIRQ(EADC0_IRQn);

    /* Begin to do ADC conversion. */
    ch = 0;

    /* TIMER_Start(TIMER0); */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;

    while (ch != 'q')
    {
        printf("Press 'q' to quit.\n");
        ch = getchar();
    }

    NVIC_DisableIRQ(TMR0_IRQn);

    /* TIMER_Close(TIMER0); */
    TIMER0->CTL = 0;
    TIMER0->EXTCTL = 0;

    /* CLK_DisableModuleClock(TMR0_MODULE); */
    CLK->APBCLK &= ~CLK_APBCLK_TMR0CKEN_Msk;

    NVIC_DisableIRQ(EADC0_IRQn);

    /* EADC_Close(EADC); */
    EADC->CTL = EADC->CTL
             | (EADC_EADC0_7 << EADC_CTL_ADC0CHSEL_Pos)                 /* Switching to channel Vss to save power */
             | ((EADC_EADC1_7-EADC_EADC1_0) << EADC_CTL_ADC1CHSEL_Pos)  /* Switching to channel Vss to save power */
             & (~EADC_CTL_ADCEN_Msk);
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    /* CLK_DisableModuleClock(EADC_MODULE); */
    CLK->APBCLK &= ~CLK_APBCLK_ADCCKEN_Msk;

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
