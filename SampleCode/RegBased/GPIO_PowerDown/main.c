/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for GPIO Power-down Wake-up feature.
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


/**
 * @brief       PortA/PortB/PortC/PortD IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortA/PortB/PortC/PortD default IRQ, declared in startup_Mini57Series.s.
 */
void GPABCD_IRQHandler(void)
{
    if(GPIO_GET_INT_FLAG(PB, BIT0)) /* To check if PB.0 interrupt occurred */
    {
        GPIO_CLR_INT_FLAG(PB, BIT0);    /* Clear PB.0 interrupt flag */
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTA, PORTB, PORTC, PORTD interrupts */
        GPIO_CLR_INT_FLAG(PA, GPIO_GET_INT_FLAG(PA, 0x3F));
        GPIO_CLR_INT_FLAG(PB, GPIO_GET_INT_FLAG(PB, 0x1F));
        GPIO_CLR_INT_FLAG(PC, GPIO_GET_INT_FLAG(PC, 0x1F));
        GPIO_CLR_INT_FLAG(PD, GPIO_GET_INT_FLAG(PD, 0x7F));
    }
}


void _CLK_PowerDown(void)
{
    /* Enable PD.0 (nRESET pin) interrupt that trigger by falling edge to make sure
       RESET button can wake up system from power down mode. */
    PD->INTTYPE &= (~BIT0);     /* edge trigger for PD0 */
    PD->INTEN   |=   BIT0;      /* enable falling or low trigger for PD0 */

    /* enable M0 register SCR[SEVONPEND] and SCR[SLEEPDEEP] */
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SEVONPEND_Msk);
    /* clear interrupt status and enable wake up interrupt */
    CLK->PWRCTL |= (CLK_PWRCTL_PDWKIF_Msk | CLK_PWRCTL_PDWKIEN_Msk);
    /* enable system power-down feature */
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);
    /* execute Wait For Interrupt; Enter power-down mode since CLK_PWRCTL_PDEN_Msk is 1 */
    __WFI();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will demonstrate how to wake up system form Power-down mode
     * by GPIO interrupt.
     */
    printf("+--------------------------------------------+\n");
    printf("| Mini57 GPIO Power Down Wake Up Sample Code |\n");
    printf("+--------------------------------------------+\n");

    printf("Please keep PB.0 low and use rising edge to wake-up system ...\n");

    /* Config multiple function to GPIO mode for PB0 */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~SYS_GPB_MFP_PB0MFP_Msk) | SYS_GPB_MFP_PB0_GPIO;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE0_Pos);

    NVIC_EnableIRQ(GP_IRQn);    /* Enable GPIO NVIC */

    /* Enable PB0 interrupt by rising edge trigger */
    PB->INTEN = (PB->INTEN & ~(GPIO_INT_BOTH_EDGE << 0)) | (GPIO_INT_RISING << 0);
    PB->INTTYPE = (PB->INTTYPE & ~(BIT0 << 0)) | (GPIO_INTTYPE_EDGE << 0);

    SYS_UnlockReg();

    /* Waiting for PB.0 rising-edge interrupt event */
    printf("Wait PB.0 to low\n");
    while(PB0 == 1);    /* wait PB.0 become low before get into power down mode */
    printf("Enter to Power-Down (rising-edge) ......  PB.0 = %d \n", PB0);

    UUART_WAIT_TX_EMPTY(UUART0);    /* To check if all the debug messages are finished */

    _CLK_PowerDown();

    printf("System waken-up done. PB.0 = %d\n\n", PB0);
    SYS_LockReg();
    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
