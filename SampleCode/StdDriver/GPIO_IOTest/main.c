/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Sample code for GPIO I/O feature.
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
uint32_t PAIntFlag=0, PBIntFlag=0, PCIntFlag=0, PDIntFlag=0;
void GPABCD_IRQHandler(void)
{
    PAIntFlag = GPIO_GET_INT_FLAG(PA, 0xF);
    PBIntFlag = GPIO_GET_INT_FLAG(PB, 0xF);
    PCIntFlag = GPIO_GET_INT_FLAG(PC, 0xF);
    PDIntFlag = GPIO_GET_INT_FLAG(PD, 0xF);

    /* clear GPIO interrupt flag */
    GPIO_CLR_INT_FLAG(PA, PAIntFlag);
    GPIO_CLR_INT_FLAG(PB, PBIntFlag);
    GPIO_CLR_INT_FLAG(PC, PCIntFlag);
    GPIO_CLR_INT_FLAG(PD, PDIntFlag);
}


int main()
{
    int32_t i32Err;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\n\nPDID 0x%08X\n", SYS_ReadPDID());    /* Display PDID */
    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will use GPIO driver to control the GPIO pin direction and
     * the high/low state, and show how to use GPIO interrupts.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 GPIO Driver Sample Code          |\n");
    printf("+-----------------------------------------+\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PB.0 and PC.2 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Config multiple function to GPIO mode for PB0 and PC2 */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~SYS_GPB_MFP_PB0MFP_Msk) | SYS_GPB_MFP_PB0_GPIO;
    SYS->GPC_MFP = (SYS->GPC_MFP & ~SYS_GPC_MFP_PC2MFP_Msk) | SYS_GPC_MFP_PC2_GPIO;

    GPIO_SetMode(PB, BIT0, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT2, GPIO_PMD_INPUT);

    GPIO_EnableInt(PC, 2, GPIO_INT_RISING);     /* Enable PC2 interrupt by rising edge trigger */
    NVIC_EnableIRQ(GP_IRQn);                    /* Enable GPIO NVIC */

    PAIntFlag = 0;
    PBIntFlag = 0;
    PCIntFlag = 0;
    PDIntFlag = 0;
    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    PB0 = 0;                /* Output low */
    CLK_SysTickDelay(10);   /* wait for IO stable */
    if (PC2 != 0)           /* check if the PB3 state is low */
    {
        i32Err = 1;
    }

    PB0 = 1;                /* Output high */
    CLK_SysTickDelay(10);   /* wait for IO stable */
    if (PC2 != 1)           /* check if the PB3 state is high */
    {
        i32Err = 1;
    }

    /* show the result */
    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure PB.0 and PC.2 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    printf("  Check Interrupt Flag PA=0x%08X, PB=0x%08X, PC=0x%08X, PD=0x%08X\n",
           PAIntFlag, PBIntFlag, PCIntFlag, PDIntFlag);

    GPIO_SetMode(PB, BIT0, GPIO_PMD_INPUT);     /* Configure PB.0 to default Input mode */
    GPIO_SetMode(PC, BIT2, GPIO_PMD_INPUT);     /* Configure PC.2 to default Input mode */

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
