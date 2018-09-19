/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:47p $
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

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_PGACKEN_Msk;

    SYS->IPRST1 |= SYS_IPRST1_PGARST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_PGARST_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initiate I/O Multi-function                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for PGA_I GPB3 */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~(SYS_GPB_MFP_PB3MFP_Msk)) | (SYS_GPB_MFP_PB3_PGA_I);

    /* The analog input port pins must be configured as input type before the PGA function is enabled. */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE3_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE3_Pos);

    /* Set GPC multi-function pins for PGA_O GPC3 */
    SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC3MFP_Msk)) | (SYS_GPC_MFP_PC3_PGA_O);

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

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will demonstrate how to amplify input signal with different gain level
     * and output to PGA_O output pin.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 PGA Sample Code                  |\n");
    printf("+-----------------------------------------+\n");

    printf("Configure PGA with gain level x2.\n");

    /* Open PGA */
    PGA->CTL = PGA_CTL_PGAEN_Msk | (PGA_GAIN_2 << PGA_CTL_GAIN_Pos);

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

    /* Close PGA */
    PGA->CTL = PGA->CTL & (~PGA_CTL_PGAEN_Msk);

    /* Disable PGA clock */
    CLK->APBCLK &= ~CLK_APBCLK_PGACKEN_Msk;

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
