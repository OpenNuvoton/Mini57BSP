/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/09 7:49p $
 * @brief
 *           Configure USCI_SPI1 as Slave mode and demonstrate how to communicate with an off-chip SPI Master device.
 *           This sample code needs to work with USCI_SPI_MasterMode sample code.
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
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_USCI1CKEN_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Set USCI_SPI1 multi-function pins */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0MFP_Msk | SYS_GPC_MFP_PC1MFP_Msk | SYS_GPC_MFP_PC2MFP_Msk | SYS_GPC_MFP_PC3MFP_Msk);
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC0_SPI1_CLK | SYS_GPC_MFP_PC1_SPI1_MISO | SYS_GPC_MFP_PC2_SPI1_MOSI | SYS_GPC_MFP_PC3_SPI1_SS);

    /* Set GPC0,2,3 as input mode and GPC1 as output mode */
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE0_Msk));
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE1_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE1_Pos);
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE2_Msk));
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE3_Msk));

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
    /* Configure USCI_SPI1 as a slave, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI1->PROTCTL |= USPI_SLAVE | USPI_MODE_0;
    USPI1->LINECTL = 0;
    /* Set USCI_SPI1 clock rate = f_PCLK */
    USPI1->BRGEN = USPI1->BRGEN & (~USPI_BRGEN_CLKDIV_Msk);
    /* Configure USCI_SPI_SS pin as low-active. */
    USPI1->CTLIN0 = (USPI1->CTLIN0 & (~USPI_CTLIN0_ININV_Msk)) | USPI_CTLIN0_ININV_Msk;
    /* Enable USCI_SPI1 protocol */
    USPI1->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;
}


int main()
{
    uint32_t u32TxDataCount, u32RxDataCount;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    /* Init USCI_SPI */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           USCI_SPI Slave Mode Sample Code           |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI1 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI1:\n");
    printf("    USCI_SPI1_SS (PC.3)\n    USCI_SPI1_CLK (PC.0)\n");
    printf("    USCI_SPI1_MISO (PC.1)\n    USCI_SPI1_MOSI (PC.2)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0xAA00 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Access TX and RX Buffer */
    while(u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if(((USPI1->BUFSTS & USPI_BUFSTS_TXFULL_Msk) == 0) && (u32TxDataCount < TEST_COUNT))
            USPI1->TXDAT = g_au32SourceData[u32TxDataCount++]; /* Write to TX Buffer */
        /* Check RX EMPTY flag */
        if((USPI1->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) == 0)
            g_au32DestinationData[u32RxDataCount++] = USPI1->RXDAT; /* Read RX Buffer */
    }

    /* Print the received data */
    printf("Received data:\n");
    for(u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }
    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Disable USCI_SPI1 function mode */
    USPI1->CTL &= ~USPI_CTL_FUNMODE_Msk;
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
