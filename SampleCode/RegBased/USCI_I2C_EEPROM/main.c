/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/29 5:16p $
 * @brief    Show how to use USCI_I2C interface to access EEPROM.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLenM;
volatile uint8_t g_u8EndFlagM = 0;
volatile enum UI2C_MASTER_EVENT m_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C1HandlerFn = NULL;

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
        UI2C1->PROTSTS = UI2C_PROTSTS_TOIF_Msk;
    }
    else
    {
        if (s_UI2C1HandlerFn != NULL)
            s_UI2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Tx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_EEPROM_MasterTx(uint32_t u32Status)
{
    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */
        UI2C_SET_DATA(UI2C1, (g_u8DeviceAddr << 1) | 0x00); /* Write SLA+W to Register TXDAT */
        m_Event = MASTER_SEND_ADDRESS;
        UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */
        if(m_Event == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C1, g_au8TxData[g_u8DataLenM++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_PTRG);
        }
        else if(m_Event == MASTER_SEND_DATA)
        {
            if(g_u8DataLenM != 3)
            {
                UI2C_SET_DATA(UI2C1, g_au8TxData[g_u8DataLenM++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_PTRG);
            }
            else
            {
                g_u8EndFlagM = 1;
                m_Event = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
            }
        }
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */
        g_u8EndFlagM = 0;
        if(m_Event == MASTER_SEND_ADDRESS)      /* SLA+W has been transmitted and NACK has been received */
        {
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_STA));        /* Send START signal */
        }
        else if(m_Event == MASTER_SEND_DATA)        /* ADDRESS has been transmitted and NACK has been received */
        {
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
        }
        else
        {
            printf("Get Wrong NACK Event\n");
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Rx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_EEPROM_MasterRx(uint32_t u32Status)
{
    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */
        if(m_Event == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C1, (g_u8DeviceAddr << 1) | 0x00); /* Write SLA+W to Register TXDAT */
            m_Event = MASTER_SEND_ADDRESS;
        }
        else if(m_Event == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C1, (g_u8DeviceAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            m_Event = MASTER_SEND_H_RD_ADDRESS;
        }
        UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */
        if(m_Event == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C1, g_au8TxData[g_u8DataLenM++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_PTRG);
        }
        else if(m_Event == MASTER_SEND_DATA)
        {
            if(g_u8DataLenM != 2)
            {
                UI2C_SET_DATA(UI2C1, g_au8TxData[g_u8DataLenM++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_PTRG);
            }
            else
            {
                m_Event = MASTER_SEND_REPEAT_START;
                UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send repeat START signal */
            }
        }
        else if(m_Event == MASTER_SEND_H_RD_ADDRESS)
        {
            m_Event = MASTER_READ_DATA;
            UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_PTRG);
        }
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */
        if(m_Event == MASTER_SEND_ADDRESS)
        {
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if(m_Event == MASTER_READ_DATA)
        {
            g_u8RxData = (unsigned char) UI2C_GET_DATA(UI2C1) & 0xFF;
            g_u8EndFlagM = 1;
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
        {
            printf("Get Wrong NACK Event\n");
        }
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

    /* Set GPC multi-function pins for USCI I2C1 GPC0(SCL) and GPC2(SDA) */
    SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC0MFP_Msk | SYS_GPC_MFP_PC2MFP_Msk)) | (SYS_GPC_MFP_PC0_I2C1_SCL | SYS_GPC_MFP_PC2_I2C1_SDA);

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

void UI2C1_Init(uint32_t u32ClkSpeed)
{
    uint32_t u32ClkDiv;
    uint32_t u32Pclk = SystemCoreClock;
    /* The USCI usage is exclusive */
    /* If user configure the USCI port as UI2C function, that port cannot use UUART or USPI function. */
    /* Open USCI_I2C1 and set clock to 100k */
    u32ClkDiv = (uint32_t) ((((((u32Pclk/2)*10)/(u32ClkSpeed))+5)/10)-1); /* Compute proper divider for USCI_I2C clock */

    /* Enable USCI_I2C protocol */
    UI2C1->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C1->CTL = 4 << UI2C_CTL_FUNMODE_Pos;

    /* Data format configuration */
    /* 8 bit data length */
    UI2C1->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C1->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;

    /* MSB data format */
    UI2C1->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    /* Set USCI_I2C bus clock */
    UI2C1->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C1->BRGEN |=  (u32ClkDiv << UI2C_BRGEN_CLKDIV_Pos);
    UI2C1->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;

    /* Set USCI_I2C1 Slave Addresses */
    UI2C1->DEVADDR0  = 0x15;    /* Slave Address : 0x15 */

    UI2C_ENABLE_PROT_INT(UI2C1, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI1_IRQn);
}

int main()
{
    uint32_t i;
    uint8_t u8Temp;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|     USCI_I2C Driver Sample Code with EEPROM 24LC64    |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init USCI_I2C1 to access EEPROM */
    UI2C1_Init(100000);

    g_u8DeviceAddr = 0x50;

    for (i = 0; i < 0x100; i++)
    {
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLenM = 0;
        g_u8EndFlagM = 0;

        /* USCI_I2C function to write data to slave */
        s_UI2C1HandlerFn = (UI2C_FUNC)USCI_I2C_EEPROM_MasterTx;

        /* USCI_I2C as master sends START signal */
        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_STA);

        /* Wait USCI_I2C Tx Finish */
        while (g_u8EndFlagM == 0);
        g_u8EndFlagM = 0;

        /* USCI_I2C function to read data from slave */
        s_UI2C1HandlerFn = (UI2C_FUNC)USCI_I2C_EEPROM_MasterRx;

        g_u8DataLenM = 0;
        g_u8DeviceAddr = 0x50;

        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_STA);

        /* Wait USCI_I2C Rx Finish */
        while (g_u8EndFlagM == 0);
        g_u8EndFlagM = 0;

        /* Compare data */
        u8Temp = g_au8TxData[2];
        if (g_u8RxData != u8Temp)
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            return -1;
        }
    }
    printf("USCI_I2C Access EEPROM Test OK\n");

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
