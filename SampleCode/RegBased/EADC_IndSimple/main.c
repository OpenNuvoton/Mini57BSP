/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for EADC Independent Simple mode.
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

    /* Select IP clock source */

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Lock protected registers */
    SYS_LockReg();
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
     * This sample code will convert EADC0 channel 0 and EADC1 channel 0 in Independent Simple mode
     * and print conversion results.
     */
    printf("+-------------------------------------------------+\n");
    printf("| Mini57 EADC Independent Simple Mode Sample Code |\n");
    printf("+-------------------------------------------------+\n");

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

    printf("Set GPB4 pin as ADC1 BIN0 input pin.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk)) | SYS_GPB_MFP_PB4_ADC1_CH0;

    /* GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT); */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE4_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE4_Pos);

    /* GPIO_DISABLE_DIGITAL_PATH(PB, BIT4); */
    PB->DINOFF |= GPIO_DINOFF_DINOFF4_Msk;

    /* Enable EADC */
    /* EADC_Open(EADC, NULL); */
    EADC->CTL |= EADC_CTL_ADCEN_Msk;

    /* Configure EADC0: channel 0 (AIN0), software trigger */
    /* EADC_ConfigSampleModule(EADC, EADC_EADC0_0, EADC_SOFTWARE_TRIGGER, NULL); */
    EADC->CTL = (EADC->CTL & ~EADC_CTL_ADC0CHSEL_Msk) | (EADC_EADC0_0 << EADC_CTL_ADC0CHSEL_Pos);
    EADC->CTL &= ~EADC_CTL_ADC0HWTRGEN_Msk;

    /* Configure EADC1: channel 0 (BIN0), software trigger */
    /* EADC_ConfigSampleModule(EADC, EADC_EADC1_0, EADC_SOFTWARE_TRIGGER, NULL); */
    EADC->CTL = (EADC->CTL & ~EADC_CTL_ADC1CHSEL_Msk) | ((EADC_EADC1_0 - EADC_EADC1_0) << EADC_CTL_ADC1CHSEL_Pos);
    EADC->CTL &= ~EADC_CTL_ADC1HWTRGEN_Msk;

    /* Configure EADC conversion mode to Independent Simple Mode */
    /* EADC_SET_INDEPENDENT_SIMPLE_MODE(EADC); */
    EADC->CTL = (EADC->CTL & ~EADC_CTL_ADCMODE_Msk) | (0x0 << EADC_CTL_ADCMODE_Pos);

    /* Configure EADC sample time to 6 EADC clocks */
    /* EADC_SetExtendSampleTime(EADC, NULL, 5); */
    EADC->SMPCNT = (EADC->SMPCNT & (~EADC_SMPCNT_ADCSMPCNT_Msk)) | ((5 & 0xF) << EADC_SMPCNT_ADCSMPCNT_Pos);

    /* Begin to do EADC conversion. */
    ch = 0;
    /* EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk|EADC_STATUS_ADC1F_Msk); */
    EADC->STATUS = (EADC_STATUS_ADC0F_Msk|EADC_STATUS_ADC1F_Msk);

    while (ch != 'q')
    {
        /* EADC_START_CONV(EADC, EADC_CTL_ADC0SWTRG_Msk); */              /* software trigger EADC0 */
        EADC->CTL |= EADC_CTL_ADC0SWTRG_Msk;

        /* while (!EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk)); */    /* Independent 2SH Mode: MUST wait EADC0 completed here. */
        while (!(EADC->STATUS & EADC_STATUS_ADC0F_Msk));

        /* EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk); */
        EADC->STATUS = EADC_STATUS_ADC0F_Msk;

        /* printf("Get EADC0 FIFO 0 data = %d\n", EADC_GET_CONV_DATA(EADC, EADC_EADC0_DAT0)); */
        printf("Get EADC0 FIFO 0 data = %d\n",
            (EADC->DAT[0] & EADC_DAT0_ADC0DAT0_Msk) >> EADC_DAT0_ADC0DAT0_Pos);

        /* EADC_START_CONV(EADC, EADC_CTL_ADC1SWTRG_Msk); */              /* software trigger EADC1 */
        EADC->CTL |= EADC_CTL_ADC1SWTRG_Msk;

        /* while (!EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC1F_Msk)); */    /* wait EADC1 completed by polling */
        while (!(EADC->STATUS & EADC_STATUS_ADC1F_Msk));

        /* EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC1F_Msk); */
        EADC->STATUS = EADC_STATUS_ADC1F_Msk;

        /* printf("Get EADC1 FIFO 0 data = %d\n", EADC_GET_CONV_DATA(EADC, EADC_EADC1_DAT0)); */
        printf("Get EADC1 FIFO 0 data = %d\n",
            (EADC->DAT[0] & EADC_DAT0_ADC1DAT0_Msk) >> EADC_DAT0_ADC1DAT0_Pos);

        printf("Press any key for next EADC conversion ... Press 'q' to quit.\n");
        ch = getchar();
    }

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
