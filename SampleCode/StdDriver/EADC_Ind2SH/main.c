/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Sample code for EADC Independent 2SH mode.
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

    /* Enable 48MHz HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC_EN);

    /* Waiting for 48MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* HCLK Clock source from HIRC */
    CLK_SetHCLK(CLK_HCLK_SRC_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable USCI0 IP clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) |
                   (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Enable IP clock */
    CLK_EnableModuleClock(EADC_MODULE);
    SYS_ResetModule(EADC_RST);

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

    printf("\n\nPDID 0x%08X\n", SYS_ReadPDID());    /* Display PDID */
    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will convert EADC0 channel 0 and EADC1 channel 0 in Independent 2SH mode
     * and print conversion results.
     */
    printf("+----------------------------------------------+\n");
    printf("| Mini57 EADC Independent 2SH Mode Sample Code |\n");
    printf("+----------------------------------------------+\n");

    /* Enable EADC clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Set EADC clock source to HIRC */
    CLK_SetModuleClock(EADC_MODULE, CLK_EADC_SRC_HIRC, CLK_CLKDIV_EADC(4));

    /* Reset IP */
    SYS_ResetModule(EADC_RST);

    printf("Set GPB0 pin as ADC0 AIN0 input pin.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB0MFP_Msk))
                   | SYS_GPB_MFP_PB0_ADC0_CH0;
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0);

    printf("Set GPB4 pin as ADC1 BIN0 input pin.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk))
                   | SYS_GPB_MFP_PB4_ADC1_CH0;
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT4);

    /* Enable EADC */
    EADC_Open(EADC, 0);

    /* Configure EADC0: channel 0 (AIN0), software trigger */
    EADC_ConfigSampleModule(EADC, EADC_EADC0_0, EADC_SOFTWARE_TRIGGER, 0);

    /* Configure EADC1: channel 0 (BIN0), software trigger */
    EADC_ConfigSampleModule(EADC, EADC_EADC1_0, EADC_SOFTWARE_TRIGGER, 0);

    /* Configure EADC conversion mode to Independent 2SH Mode */
    EADC_SET_INDEPENDENT_2SH_MODE(EADC);

    /* Configure EADC sample time to 6 EADC clocks */
    EADC_SetExtendSampleTime(EADC, 0, 5);

    /* Begin to do EADC conversion. */
    ch = 0;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk|EADC_STATUS_ADC1F_Msk);
    while (ch != 'q')
    {
        EADC_START_CONV(EADC, EADC_CTL_ADC0SWTRG_Msk);              /* software trigger EADC0 */
        EADC_START_CONV(EADC, EADC_CTL_ADC1SWTRG_Msk);              /* software trigger EADC1 */
        while (!EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk));    /* Independent 2SH Mode: MUST wait EADC0 completed here. */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk);
        printf("Get EADC0 FIFO 0 data = %lu\n", EADC_GET_CONV_DATA(EADC, EADC_EADC0_DAT0));
        printf("Get EADC1 FIFO 0 data = %lu\n", EADC_GET_CONV_DATA(EADC, EADC_EADC1_DAT0));

        printf("Press any key for next EADC conversion ... Press 'q' to quit.\n");
        ch = getchar();
    }

    EADC_Close(EADC);
    CLK_DisableModuleClock(EADC_MODULE);
    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
