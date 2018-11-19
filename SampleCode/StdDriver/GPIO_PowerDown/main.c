/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
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

    /* Lock protected registers */
    SYS_LockReg();
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
    if(GPIO_GET_INT_FLAG(PB, BIT0))     /* To check if PB.0 interrupt occurred */
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


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\n\nPDID 0x%08X\n", SYS_ReadPDID());    /* Display PDID */
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
    GPIO_SetMode(PB, BIT0, GPIO_PMD_INPUT);
    NVIC_EnableIRQ(GP_IRQn);                    /* Enable GPIO NVIC */
    GPIO_EnableInt(PB, 0, GPIO_INT_RISING);     /* Enable PB0 interrupt by rising edge trigger */

    SYS_UnlockReg();
    /* Waiting for PB.0 rising-edge interrupt event */
    printf("Wait PB.0 to low\n");
    while(PB0 == 1);    /* wait PB.0 become low before get into power down mode */
    printf("Enter to Power-Down (rising-edge) ......  PB.0 = %d \n", PB0);
    UUART_WAIT_TX_EMPTY(UUART0);  /* To check if all the debug messages are finished */

    CLK_PowerDown();

    printf("System waken-up done. PB.0 = %d\n\n", PB0);
    SYS_LockReg();
    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
