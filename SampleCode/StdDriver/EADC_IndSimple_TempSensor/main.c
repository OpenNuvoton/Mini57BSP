/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Sample code for EADC Independent Simple mode with Temperature Sensor input.
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
     * This sample code will convert EADC1 channel 6 (Temperature Sensor) in Independent Simple mode
     * and print conversion results.
     */
    printf("+------------------------------------------------+\n");
    printf("| Mini57 EADC for Temperature Sensor Sample Code |\n");
    printf("+------------------------------------------------+\n");

    /* Enable EADC clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Set EADC clock source to HIRC */
    CLK_SetModuleClock(EADC_MODULE, CLK_EADC_SRC_HIRC, CLK_CLKDIV_EADC(4));

    /* Reset IP */
    SYS_ResetModule(EADC_RST);

    /* Enable EADC */
    EADC_Open(EADC, NULL);

    /* Configure EADC1: channel 6 (Temperature Sensor), software trigger */
    EADC_ConfigSampleModule(EADC, EADC_EADC1_6, EADC_SOFTWARE_TRIGGER, NULL);
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;  /* Enable Temperature Sensor */

    /* Configure EADC conversion mode to Independent Simple Mode */
    EADC_SET_INDEPENDENT_SIMPLE_MODE(EADC);

    /* Configure EADC sample time to 6 EADC clocks */
    EADC_SetExtendSampleTime(EADC, NULL, 5);

    /* Begin to do EADC conversion. */
    ch = 0;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC1F_Msk);
    while (ch != 'q')
    {
        EADC_START_CONV(EADC, EADC_CTL_ADC1SWTRG_Msk);              /* software trigger EADC1 */
        while (!EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC1F_Msk));    /* wait EADC1 completed by polling */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC1F_Msk);
        printf("Get ADC1 FIFO 0 data = %d\n", EADC_GET_CONV_DATA(EADC, EADC_EADC1_DAT0));

        printf("Press any key for next EADC conversion ... Press 'q' to quit.\n");
        ch = getchar();
    }

    EADC_Close(EADC);
    CLK_DisableModuleClock(EADC_MODULE);
    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/