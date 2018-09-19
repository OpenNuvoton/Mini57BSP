/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Sample code for EADC Simultaneous Sequential 3R mode.
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
    uint32_t dataVaidFlag;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\n\nPDID 0x%08X\n", SYS_ReadPDID());    /* Display PDID */
    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will convert EADC0 channel 0, channel 6, and EADC1 channel 0 in
     * EADC Simultaneous Sequential 3R mode and print conversion results.
     */
    printf("+---------------------------------------------------------+\n");
    printf("| Mini57 EADC Simultaneous Sequential 3R Mode Sample Code |\n");
    printf("+---------------------------------------------------------+\n");

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

    /* Configure EADC conversion mode to Simultaneous Sequential 3R Mode and use EADC0 channel 6 (Band-Gap) as second input. */
    printf("Set EADC0 second input to channel 6 Band gap.\n");
	EADC_SET_SIMULTANEOUS_3R_MODE(EADC, EADC_EADC0_6);

    /* Configure EADC sample time to 6 EADC clocks */
    EADC_SetExtendSampleTime(EADC, 0, 5);

    /* Begin to do EADC conversion. */
    ch = 0;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk|EADC_STATUS_ADC1F_Msk);
    while (ch != 'q')
    {
        EADC_START_CONV(EADC, EADC_CTL_ADC0SWTRG_Msk);              /* software trigger EADC0 */
        while (!EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk));    /* Simultaneous Sequential 3R Mode: wait EADC0 completed here for both EADC0 and EADC1 conversion. */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk);

	    /* MUST read and keep valid flag in DAT register before any others read it. */
        dataVaidFlag = EADC_GET_DATA_VALID_FLAG(EADC, EADC_BIT_MASK_EADC0 | EADC_BIT_MASK_EADC1 | EADC_BIT_MASK_EADC0_DAT1 | EADC_BIT_MASK_EADC1_DAT1);

        printf("Get EADC0 FIFO 0 data = %4d (valid = %s)\n", EADC_GET_CONV_DATA(EADC, EADC_EADC0_DAT0), (dataVaidFlag & EADC_BIT_MASK_EADC0) ? "Yes" : "No");
        /* Simultaneous Sequential 3R Mode : EADC1 conversion is valid that don't need to trigger EADC1 */
        printf("Get EADC1 FIFO 0 data = %4d (valid = %s)\n", EADC_GET_CONV_DATA(EADC, EADC_EADC1_DAT0), (dataVaidFlag & EADC_BIT_MASK_EADC1) ? "Yes" : "No");
        /* Simultaneous Sequential 3R Mode : FIFO 1 for EADC0 conversion are valid */
        printf("Get EADC0 FIFO 1 data = %4d (valid = %s)\n", EADC_GET_CONV_DATA(EADC, EADC_EADC0_DAT1), (dataVaidFlag & EADC_BIT_MASK_EADC0_DAT1) ? "Yes" : "No");
        /* Simultaneous Sequential 3R Mode : FIFO 1 for EADC1 conversion are NOT valid */
        printf("Get EADC1 FIFO 1 data = %4d (valid = %s)\n", EADC_GET_CONV_DATA(EADC, EADC_EADC1_DAT1), (dataVaidFlag & EADC_BIT_MASK_EADC1_DAT1) ? "Yes" : "No");

        printf("Press any key for next EADC conversion ... Press 'q' to quit.\n");
        ch = getchar();
    }

    EADC_Close(EADC);
    CLK_DisableModuleClock(EADC_MODULE);
    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
