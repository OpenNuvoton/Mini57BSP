/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"

/*----------------------------------------------------------*/
/*  Function for disable internal analog POR circuit        */
/*----------------------------------------------------------*/
void SYS_Disable_AnalogPORCircuit(void)
{
    SYS->PORCTL = 0x5AA5;
}

/*----------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode            */
/*----------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UUART0->PROTSTS = (UUART_PROTSTS_TXENDIF_Msk | UUART_PROTSTS_TXSTIF_Msk);
    while(!((UUART0->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk) >> UUART_BUFSTS_TXEMPTY_Pos));
    if(UUART0->PROTSTS & UUART_PROTSTS_TXSTIF_Msk)
        while((UUART0->PROTSTS & UUART_PROTSTS_TXENDIF_Msk) != UUART_PROTSTS_TXENDIF_Msk);

    /* Enter to Power-down mode */
    /* enable M0 register SCR[SEVONPEND] and SCR[SLEEPDEEP] */
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SEVONPEND_Msk);
    /* clear interrupt status and enable wake up interrupt */
    CLK->PWRCTL |= (CLK_PWRCTL_PDWKIF_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    /* enable system power-down feature */
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);
    /* execute Wait For Interrupt; Enter power-down mode since CLK_PWRCTL_PDEN_Msk is 1 */
    __WFI();
}

void GPABCD_IRQHandler(void)
{
    volatile uint32_t temp;

    /* To check if PB.3 interrupt occurred */
    if (PB->INTSRC & BIT3)
    {
        /* To clear PB.3 interrupt flag */
        PB->INTSRC = BIT3;
        printf("PB.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        temp = PB->INTSRC;
        PB->INTSRC = temp;
        printf("Un-expected interrupts.\n");
    }
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

    /* Enable USCI0 IP clock */
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_USCI0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) |
                   (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | 
               (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos) |
               (GPIO_MODE_INPUT << GPIO_MODE_MODE6_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Set UART function mode */
    UUART0->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART0->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART0->BRGEN = (51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos); /* Set UART baud rate as 115200bps */
    UUART0->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */

    printf("\n\nPDID 0x%08lX\n", SYS->PDID & SYS_PDID_PDID_Msk);     /* Display PDID */
    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    printf("+-------------------------------------------------------------------+\n");
    printf("| Mini57 SYS_PowerDown_MinCurrent and Wake-up by PB.3 Sample Code   |\n");
    printf("+-------------------------------------------------------------------+\n");

    printf("+-------------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                      |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                               |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                      |\n");
    printf("|  3. Must enable LVR                                                     |\n");
    printf("|  4. Disable analog function, e.g. ADC, ACMP, and POR module.            |\n");
    printf("|  5. Enter to Power-Down                                                 |\n");
    printf("|  6. Wait for PB.3 falling-edge interrupt event to wakeup the MCU        |\n");
    printf("+-------------------------------------------------------------------------+\n\n");

    /* Check if all the debug messages are finished */
    UUART0->PROTSTS = (UUART_PROTSTS_TXENDIF_Msk | UUART_PROTSTS_TXSTIF_Msk);
    while(!((UUART0->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk) >> UUART_BUFSTS_TXEMPTY_Pos));
    if(UUART0->PROTSTS & UUART_PROTSTS_TXSTIF_Msk)
        while((UUART0->PROTSTS & UUART_PROTSTS_TXENDIF_Msk) != UUART_PROTSTS_TXENDIF_Msk);

    /* Configure all GPIO as Quasi-bidirectional Mode */
    PA->MODE = 0xFFFFFFFF;
    PB->MODE = 0xFFFFFFFF;
    PC->MODE = 0xFFFFFFFF;
    PD->MODE = 0xFFFFFFFF;

    /* Configure PB.3 as Quasi-bidirectional mode */
    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE3_Msk)) |
               (GPIO_MODE_QUASI << GPIO_MODE_MODE3_Pos);

    /* Enable PB.3 interrupt by falling edge trigger */
    PB->INTEN = (PB->INTEN & ~(GPIO_INT_BOTH_EDGE << 3)) | (GPIO_INT_FALLING << 3);
    PB->INTTYPE = (PB->INTTYPE & ~(BIT0 << 3)) | (GPIO_INTTYPE_EDGE << 3);

    NVIC_EnableIRQ(GP_IRQn);

    /* Select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO->GPIO_DBCTL = (GPIO_DBCTL_ICLKON_Msk | GPIO_DBCTL_DBCLKSRC_LIRC | GPIO_DBCTL_DBCLKSEL_1024);

    /* Enable interrupt de-bounce function */
    PB->DBEN |= BIT3;

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* LIRC must be disabled */
    CLK->PWRCTL &= ~CLK_PWRCTL_LIRCEN_Msk;

    /* LVR must be enabled */
    SYS->BODCTL |= SYS_BODCTL_LVREN_Msk;

    /* BOD must be disabled */
    SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk;

    /* Turn off internal analog POR circuit */
    SYS_Disable_AnalogPORCircuit();

    /* Disable Power-on Reset */
    SYS->PORCTL = 0x5AA5;

    printf("Enter to Power-Down ......\n");
    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Waiting for PB.3 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

    while(1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
