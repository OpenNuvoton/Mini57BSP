/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 17/08/29 1:46p $
 * @brief    Show how to wake-up USCI_I2C from deep sleep mode.
 *           This sample code needs to work with USCI_I2C_Master.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8SlvData[256];
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8RxData[4];
volatile uint16_t g_u16RecvAddr;
volatile uint8_t g_u8DataLenS;
volatile uint8_t g_u8SlvPWRDNWK = 0, g_u8SlvI2CWK = 0;
volatile uint32_t g_u32WKfromAddr;

volatile enum UI2C_SLAVE_EVENT s_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C1HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    /* Check system power down mode wake-up interrupt flag */
    if(((CLK->PWRCTL) & CLK_PWRCTL_PDWKIF_Msk) != 0)
    {
        /* Clear system power down wake-up interrupt flag */
        CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
        g_u8SlvPWRDNWK = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C1 IRQ Handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void USCI1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = UI2C_GET_PROT_STATUS(UI2C1);

    if (UI2C_GET_TIMEOUT_FLAG(UI2C1))
    {
        /* Clear USCI_I2C1 Timeout Flag */
        UI2C_ClearTimeoutFlag(UI2C1);
    }
    else
    {
        if (s_UI2C1HandlerFn != NULL)
            s_UI2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C toggle wake-up                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_SLV_Toggle_Wakeup(uint32_t u32Status)
{
    uint32_t temp;

    if((UI2C1->WKSTS & UI2C_WKSTS_WKF_Msk) == UI2C_WKSTS_WKF_Msk)
    {
        g_u32WKfromAddr = 0;
        g_u8SlvI2CWK = 1;

        /* Clear WKF INT Flag */
        UI2C_CLR_WAKEUP_FLAG(UI2C1);
        return;
    }

    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if(s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8DataLenS = 0;

            if((UI2C1->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C1, g_au8SlvData[slave_buff_addr]);
                slave_buff_addr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }
            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if(s_Event == SLAVE_GET_DATA)
        {
            temp = (uint8_t)UI2C_GET_DATA(UI2C1);
            g_au8RxData[g_u8DataLenS] = temp;
            g_u8DataLenS++;

            if(g_u8DataLenS == 2)
            {
                /* Address has been received; ACK has been returned*/
                temp = (g_au8RxData[0] << 8);
                temp += g_au8RxData[1];
                slave_buff_addr = temp;
            }
            if(g_u8DataLenS == 3)
            {
                temp = g_au8RxData[2];
                g_au8SlvData[slave_buff_addr] = temp;
                g_u8DataLenS = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STORIF_Msk);

        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C address match wake-up                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_SLV_Address_Wakeup(uint32_t u32Status)
{
    uint32_t temp;

    if((UI2C1->WKSTS & UI2C_WKSTS_WKF_Msk) == UI2C_WKSTS_WKF_Msk)
    {
        g_u32WKfromAddr = 1;
        g_u8SlvI2CWK = 1;

        while((UI2C1->PROTSTS & UI2C_PROTSTS_WKAKDONE_Msk) == 0) {};

        /* Clear WK flag */
        UI2C_CLR_WAKEUP_FLAG(UI2C1);
        UI2C1->PROTSTS = UI2C_PROTSTS_WKAKDONE_Msk;

        return;
    }

    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if(s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8DataLenS = 0;

            if((UI2C1->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C1, g_au8SlvData[slave_buff_addr]);
                slave_buff_addr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }
            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if(s_Event == SLAVE_GET_DATA)
        {
            temp = (uint8_t)UI2C_GET_DATA(UI2C1);
            g_au8RxData[g_u8DataLenS] = temp;
            g_u8DataLenS++;

            if(g_u8DataLenS == 2)
            {
                /* Address has been received; ACK has been returned*/
                temp = (g_au8RxData[0] << 8);
                temp += g_au8RxData[1];
                slave_buff_addr = temp;
            }
            if(g_u8DataLenS == 3)
            {
                temp = g_au8RxData[2];
                g_au8SlvData[slave_buff_addr] = temp;
                g_u8DataLenS = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STORIF_Msk);

        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

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
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Set GPC multi-function pins for USCI I2C1 GPC0(SCL) and GPC2(SDA) */
    SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC0MFP_Msk | SYS_GPC_MFP_PC2MFP_Msk)) | (SYS_GPC_MFP_PC0_I2C1_SCL | SYS_GPC_MFP_PC2_I2C1_SDA);

    /* Lock protected registers */
    SYS_LockReg();
}

void UI2C1_Init(void)
{
    /* The USCI usage is exclusive */
    /* If user configure the USCI port as UI2C function, that port cannot use UUART or USPI function. */
    /* Open USCI_I2C1 and set clock to 100k */
    UI2C_Open(UI2C1, 100000);

    /* Get USCI_I2C1 Bus Clock */
    printf("USCI_I2C clock %d Hz\n", UI2C_GetBusClockFreq(UI2C1));

    /* Set USCI_I2C1 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C1, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */

    /* Set USCI_I2C1 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C1, 0, 0x01);

    UI2C_ENABLE_PROT_INT(UI2C1, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI1_IRQn);
}

int main()
{
    uint32_t i, temp;
    uint8_t  ch;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("| USCI_I2C Driver Sample Code (Slave) for wake-up & access Slave test |\n");
    printf("| Needs to work with USCI_I2C_Master sample code.                     |\n");
    printf("|      UI2C Master (I2C1) <---> UI2C Slave (I2C1)                     |\n");
    printf("| !! This sample code requires two borads for testing !!              |\n");
    printf("+---------------------------------------------------------------------+\n");

    /* Init USCI_I2C1 */
    UI2C1_Init();

    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("[T] I/O Toggle Wake-up Mode\n");
    printf("[A] Address Match Wake-up Mode\n");
    printf("Select: ");
    ch =  getchar();

    if((ch == 'T') || (ch == 't'))
    {
        printf("(T)oggle\n");

        /* Enable UI2C1 toggle mode wake-up */
        UI2C_EnableWakeup(UI2C1, UI2C_DATA_TOGGLE_WK);
        s_Event = SLAVE_ADDRESS_ACK;

        /* UI2C1 function to Slave receive/transmit data */
        s_UI2C1HandlerFn = USCI_I2C_SLV_Toggle_Wakeup;
    }
    else
    {
        /* Default Mode*/
        printf("(A)ddress math\n");

        /* Enable UI2C1 address match mode wake-up */
        UI2C_EnableWakeup(UI2C1, UI2C_ADDR_MATCH_WK);
        s_Event = SLAVE_GET_DATA;

        /* UI2C1 function to Slave receive/transmit data */
        s_UI2C1HandlerFn = USCI_I2C_SLV_Address_Wakeup;
    }

    UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_AA);

    for(i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* Enable power wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);

    /* System power down enable */
    printf("\nCHIP enter power down status.\n");

    /* Wait USCI UART buffer empty to get a cleaner console out */
    UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_TXENDIF_Msk | UUART_PROTSTS_TXSTIF_Msk);
    while(!UUART_IS_TX_EMPTY(UUART0));
    if(UUART_GET_PROT_STATUS(UUART0) & UUART_PROTSTS_TXSTIF_Msk)
        while((UUART_GET_PROT_STATUS(UUART0) & UUART_PROTSTS_TXENDIF_Msk) != UUART_PROTSTS_TXENDIF_Msk);

    /* Clear flage before enter power-down mode */
    if(UI2C1->PROTSTS != 0)
    {
        temp = UI2C1->PROTSTS;
        UI2C1->PROTSTS = temp;
    }

    CLK_PowerDown();

    while(g_u8SlvPWRDNWK == 0);
    while(g_u8SlvI2CWK == 0);

    if(g_u32WKfromAddr)
        printf("UI2C1 [A]ddress match Wake-up from Deep Sleep\n");
    else
        printf("UI2C1 [T]oggle Wake-up from Deep Sleep\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
