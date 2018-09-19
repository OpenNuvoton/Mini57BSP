/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/24 3:34p $
 * @brief
 *           Show how to use auto baud rate detection function.
 *           This sample code needs to work with USCI_UART_AutoBaudRate_Slave.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_TxTest(void);

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

    /* Set GPD multi-function pins for USCI UART1 GPD3(TX) and GPD4(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD3MFP_Msk | SYS_GPD_MFP_PD4MFP_Msk)) | (SYS_GPD_MFP_PD3_UART1_TXD | SYS_GPD_MFP_PD4_UART1_RXD);

    /* Set GPD3 as output mode and GPD4 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE3_Msk | GPIO_MODE_MODE4_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE3_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

void UUART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS->IPRST1 |=  SYS_IPRST1_USCI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_USCI0RST_Msk;

    /* Configure USCI0 as UART mode */
    UUART0->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Set UART function mode */
    UUART0->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART0->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART0->BRGEN = (51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos); /* Set UART baud rate as 115200bps */
    UUART0->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */
}

void UUART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI1 */
    SYS->IPRST1 |=  SYS_IPRST1_USCI1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_USCI1RST_Msk;

    /* Configure USCI1 as UART mode */
    UUART1->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Set UART function mode */
    UUART1->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART1->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART1->BRGEN = (51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos); /* Set UART baud rate as 115200bps */
    UUART1->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */
}

int main()
{
    SYS_Init();

    /* The USCI usage is exclusive */
    /* If user configure the USCI port as UUART function, that port cannot use USPI or UI2C function. */
    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    /* The USCI usage is exclusive */
    /* If user configure the USCI port as UUART function, that port cannot use USPI or UI2C function. */
    /* Init USCI UART1 for test */
    UUART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART auto baud rate sample master function */
    USCI_AutoBaudRate_TxTest();

    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test Menu                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_TestItem(void)
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test (Master)                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] baud rate 38400 bps                                   |\n");
    printf("| [2] baud rate 57600 bps                                   |\n");
    printf("| [3] baud rate 115200 bps                                  |\n");
    printf("|                                                           |\n");
    printf("| Select baud rate and master will send 0x55 to slave ...   |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_TxTest(void)
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|                                          |Slave| |\n");
    printf("| |    TX|--USCI1_DAT1(PD.3) <==> USCI1_DAT0(PD.4)--|RX   | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x55 with different baud rate. It can check if Slave   |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");

    do
    {
        USCI_AutoBaudRate_TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch(u32Item)
        {
        case '1':
            UUART1->BRGEN = (103 << UUART_BRGEN_CLKDIV_Pos) | (5 << UUART_BRGEN_DSCNT_Pos) | (1 << UUART_BRGEN_PDSCNT_Pos);
            break;
        case '2':
            UUART1->BRGEN = (51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos) | (1 << UUART_BRGEN_PDSCNT_Pos);
            break;
        default:
            UUART1->BRGEN = (51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos);
            break;
        }

        /* Send input pattern 0x55 for auto baud rate detection */
        UUART1->TXDAT = 0x55;

    }
    while(u32Item != 27);

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
