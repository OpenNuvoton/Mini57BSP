/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 17/05/04 5:20p $
 * @brief    Sample code for PGA output to PGA_O pin.
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
    CLK_EnableModuleClock(PGA_MODULE);
    SYS_ResetModule(PGA_RST);

    /* Select IP clock source */

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initiate I/O Multi-function                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for PGA_I GPB3 */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~(SYS_GPB_MFP_PB3MFP_Msk))
                    | (SYS_GPB_MFP_PB3_PGA_I);
    /* The analog input port pins must be configured as input type before the PGA function is enabled. */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);

    /* Set GPC multi-function pins for PGA_O GPC3 */
    SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC3MFP_Msk))
                    | (SYS_GPC_MFP_PC3_PGA_O);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Initiate USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\n\nPDID 0x%08X\n", SYS_ReadPDID());    /* Display PDID */
    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will demonstrate how to amplify input signal with different gain level
     * and output to PGA_O output pin.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 PGA Sample Code                  |\n");
    printf("+-----------------------------------------+\n");

    printf("Configure PGA with gain level x2.\n");
    PGA_Open(PGA, PGA_GAIN_2, 1);
    printf("Compare the signal on input pin GPB3 and output pin GPC3.\n");
    printf("Press any key to next gain level ... \n");  getchar();

    printf("Configure PGA with gain level x1.\n");
    PGA_SET_GAIN(PGA, PGA_GAIN_1);
    printf("Compare the signal on input pin GPB3 and output pin GPC3.\n");
    printf("Press any key to next gain level ... \n");  getchar();

    printf("Configure PGA with gain level x5.\n");
    PGA_SET_GAIN(PGA, PGA_GAIN_5);
    printf("Compare the signal on input pin GPB3 and output pin GPC3.\n");
    printf("Press any key to next gain level ... \n");  getchar();

    printf("Configure PGA with gain level x7.\n");
    PGA_SET_GAIN(PGA, PGA_GAIN_7);
    printf("Compare the signal on input pin GPB3 and output pin GPC3.\n");
    printf("Press any key to disable PGA_O pin ... \n");  getchar();

    printf("Configure PGA to disable PGA_O pin.\n");
    PGA_DISABLE_OUTPUT_PGAO(PGA);
    printf("The output pin GPC3 should has no signal.\n");
    printf("Press any key to close PGA and exit ... \n");  getchar();

    PGA_Close(PGA);
    CLK_DisableModuleClock(PGA_MODULE);
    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
