/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:47p $
 * @brief    Sample code for HDIV calculation feature.
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

    /* Enable IP clock */
    CLK->AHBCLK |= CLK_AHBCLK_HDIVCKEN_Msk;
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
    int32_t x, y, q, r;
    int i;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will demonstrate how to divide two signed integer by HDIV engine.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 HDIV Sample Code                 |\n");
    printf("+-----------------------------------------+\n");

    x = 123456;
    y = 21;
    for (i=0; i<5; i++)
    {
        q = HDIV_Div(x, y);
        r = HDIV_Mod(x, y);

        if (! HDIV_IS_DIVBYZERO())
        {
            printf("HDIV calculation : %d / %d = %d ... %d, Divide by 0 flag : %d\n", x, y, q, r, HDIV_IS_DIVBYZERO());
            printf("Correct answer   : %d / %d = %d ... %d\n", x, y, x / y, x % y);
            if ((q == x / y) && (r == x % y))
                printf("==> Division PASS !!!\n");
            else
                printf("==> Division FAIL !!!\n");
        }
        else
        {
            printf("HDIV calculation : %d / %d, divide by 0 !!!\n", x, y);
            printf("==> Divide by zero checking PASS !!!\n");
        }

        y -= 7;
        printf("\n");
    }

    CLK->AHBCLK &= ~CLK_AHBCLK_HDIVCKEN_Msk;

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
