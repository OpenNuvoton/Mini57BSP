/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief
 *           Implement USCI_SPI1 Master loop back transfer.
 *           This sample code needs to connect USCI_SPI1_MISO pin and USCI_SPI1_MOSI pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


#define TEST_COUNT             64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable 48MHz HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for 48MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* HCLK Clock source from HIRC */
    CLK_SetHCLK(CLK_HCLK_SRC_HIRC,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(USCI1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Set USCI_SPI1 multi-function pins */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0MFP_Msk | SYS_GPC_MFP_PC1MFP_Msk | SYS_GPC_MFP_PC2MFP_Msk | SYS_GPC_MFP_PC3MFP_Msk);
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC0_SPI1_CLK | SYS_GPC_MFP_PC1_SPI1_MISO | SYS_GPC_MFP_PC2_SPI1_MOSI | SYS_GPC_MFP_PC3_SPI1_SS);

    /* Set GPC0,2,3 as output mode and GPC1 as Input mode */	
    GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT1, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);

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
    /* Configure as a master, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set USCI_SPI clock rate = 2MHz */
    USPI_Open(USPI1, USPI_MASTER, USPI_MODE_0, 16, 2000000);

}


int main()
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* Init USCI_SPI */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|                   USCI_SPI Driver Sample Code                    |\n");
    printf("+------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates USCI_SPI1 self loop back data transfer.\n");
    printf(" USCI_SPI1 configuration:\n");
    printf("     Master mode; data width 16 bits.\n");
    printf(" I/O connection:\n");
    printf("     PC.2 USCI_SPI1_MOSI <--> PC.1 USCI_SPI1_MISO \n");

    printf("\nUSCI_SPI1 Loopback test ");

    /* set the source data and clear the destination buffer */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++) {
        g_au32SourceData[u32DataCount] = u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    u32Err = 0;
    for(u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++) {
        /* set the source data and clear the destination buffer */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++) {
            g_au32SourceData[u32DataCount]++;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if((u32TestCount & 0x1FF) == 0) {
            putchar('.');
        }

        while(1) {
            /* Write to TX register */
            USPI_WRITE_TX(USPI1, g_au32SourceData[u32DataCount]);
            /* Check USCI_SPI1 busy status */
            while(USPI_IS_BUSY(USPI1));
            /* Read received data */
            g_au32DestinationData[u32DataCount] = USPI_READ_RX(USPI1);
            u32DataCount++;
            if(u32DataCount == TEST_COUNT)
                break;
        }

        /*  Check the received data */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++) {
            if(g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Close USCI_SPI1 */
    USPI_Close(USPI1);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
