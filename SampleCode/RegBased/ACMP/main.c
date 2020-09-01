/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/08 9:53a $
 * @brief    Demonstrate analog comparator (ACMP) comparison by comparing
 *           ACMP0_P0 input and VBG voltage and shows the result on UART console
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"

void ACMP_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP, 0);
    /* Check Comparator 0 Output Status */
    if(ACMP_GET_OUTPUT(ACMP, 0))
        printf("ACMP0_P0 voltage > Band-gap voltage (%d)\n", u32Cnt);
    else
        printf("ACMP0_P0 voltage <= Band-gap voltage (%d)\n", u32Cnt);

    u32Cnt++;
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
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_ACMPCKEN_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Set GPB0 multi-function pin for ACMP0 positive input pin */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~SYS_GPB_MFP_PB0MFP_Msk) | SYS_GPB_MFP_PB0_ACMP0_P0;

    /* Set GPB0 as input mode */
    PB->MODE = PB->MODE & ~(GPIO_MODE_MODE0_Msk);

    /* Disable digital input path of analog pin ACMP0_P to prevent leakage */
    PB->DINOFF |= GPIO_DINOFF_DINOFF0_Msk;

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

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|         ACMP Sample Code              |\n");
    printf("+---------------------------------------+\n");

    printf("\nThis sample code demonstrates ACMP0 function. Using ACMP0_P0 (PB.0) as ACMP0\n");
    printf("positive input and using internal band-gap voltage as the negative input\n");

    /* Configure ACMP0. Enable ACMP0 and select internal reference voltage as negative input. */
    ACMP->CTL[0] = (ACMP->CTL[0] & ~(ACMP_CTL_CPNSEL_Msk | ACMP_CTL_ACMPHYSEN_Msk)) |
                   (ACMP_CTL_NEGSEL_VBG | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_ACMPEN_Msk);
    /* Set ACMP0 rising edge and falling edge as interrupt condition. */
    ACMP->CTL[0] = (ACMP->CTL[0] & ~ACMP_CTL_EDGESEL_Msk) | ACMP_CTL_INTPOL_RF;
    /* Enable ACMP0 interrupt function */
    ACMP->CTL[0] |= ACMP_CTL_ACMPIE_Msk;

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP_IRQn);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
