/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/09 7:45p $
 * @brief
 *          Configure USCI_SPI1 as Master mode and demonstrate how to communicate with an off-chip SPI Slave device.
 *          Needs to work with USCI_SPI_SlaveMode sample code.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

void USCI1_IRQHandler(void)
{
    uint32_t u32RxData;

    /* Clear TX end interrupt flag */
    USPI1->PROTSTS = USPI_PROTSTS_TXENDIF_Msk;

    /* Check RX EMPTY flag */
    while((USPI1->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) == 0) {
        /* Read RX Buffer */
        u32RxData = USPI1->RXDAT;
        g_au32DestinationData[g_u32RxDataCount++] = u32RxData;
    }
    /* Check TX data count */
    if(g_u32TxDataCount < TEST_COUNT) {
        /* Write to TX Buffer */
        USPI1->TXDAT = g_au32SourceData[g_u32TxDataCount++];
    }
}


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

    /* Enable IP clock */
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_USCI1CKEN_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Set USCI_SPI1 multi-function pins */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0MFP_Msk | SYS_GPC_MFP_PC1MFP_Msk | SYS_GPC_MFP_PC2MFP_Msk | SYS_GPC_MFP_PC3MFP_Msk);
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC0_SPI1_CLK | SYS_GPC_MFP_PC1_SPI1_MISO | SYS_GPC_MFP_PC2_SPI1_MOSI | SYS_GPC_MFP_PC3_SPI1_SS);

    /* Set GPC0,2,3 as output mode and GPC1 as Input mode */
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE0_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE1_Msk));
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE2_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE3_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE3_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI                                                                                           */
    /*---------------------------------------------------------------------------------------------------------*/
    /* The USCI usage is exclusive */
    /* If user configure the USCI port as USPI function, that port cannot use UUART or UI2C function. */
    /* Select USCI_SPI1 protocol */
    USPI1->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI1->CTL = 1 << USPI_CTL_FUNMODE_Pos;
    /* Configure USCI_SPI1 as a master, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI1->PROTCTL |= USPI_MASTER | USPI_MODE_0;
    USPI1->LINECTL = 0;
    /* Set USCI_SPI1 clock rate = f_PCLK / 2*(11+1) = 2 MHz */
    USPI1->BRGEN = (USPI1->BRGEN & (~USPI_BRGEN_CLKDIV_Msk)) | (0x0B << USPI_BRGEN_CLKDIV_Pos);
    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
    USPI1->LINECTL = (USPI1->LINECTL & (~USPI_LINECTL_CTLOINV_Msk)) | USPI_SS_ACTIVE_LOW;
    USPI1->PROTCTL |= USPI_PROTCTL_AUTOSS_Msk;
    /* Enable USCI_SPI1 protocol */
    USPI1->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;
}


int main()
{
    uint32_t u32DataCount;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* Init USCI_SPI */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|             USCI_SPI Master Mode Sample Code           |\n");
    printf("+--------------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI1 as a master.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI1:\n");
    printf("    USCI_SPI1_SS (PC.3)\n    USCI_SPI1_CLK (PC.0)\n");
    printf("    USCI_SPI1_MISO (PC.1)\n    USCI_SPI1_MOSI (PC.2)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The USCI_SPI master configuration is ready.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++) {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x5500 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.");
    getchar();
    printf("\n");

    /* Enable TX end interrupt */
    USPI1->INTEN |= USPI_INTEN_TXENDIEN_Msk;
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(USCI1_IRQn);

    /* Write to TX Buffer */
    USPI1->TXDAT = g_au32SourceData[g_u32TxDataCount++];

    /* Wait for transfer done */
    while(g_u32RxDataCount < TEST_COUNT);

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++) {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable TX end interrupt */
    USPI1->INTEN &= ~USPI_INTEN_TXENDIEN_Msk;
    NVIC_DisableIRQ(USCI1_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Disable USCI_SPI1 function mode */
    USPI1->CTL &= ~USPI_CTL_FUNMODE_Msk;
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
