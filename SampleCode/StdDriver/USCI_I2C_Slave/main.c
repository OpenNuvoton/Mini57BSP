/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 17/08/29 10:32a $
 * @brief    Show how to set USCI_I2C in Slave mode and receive the data from Master.
 *           This sample code needs to work with USCI_I2C_Master.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8RxData[3];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8DataLenS;
volatile enum UI2C_SLAVE_EVENT s_Event;

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
        UI2C_ClearTimeoutFlag(UI2C1);
    }
    else
    {
        if (s_UI2C1HandlerFn != NULL)
            s_UI2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C TRx Callback Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_SlaveTRx(uint32_t u32Status)
{
    uint32_t temp;

    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */
        if(s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8DataLenS = 0;
            if((u32Status & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)     /* Own SLA+R has been receive; ACK has been return */
            {
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C1, g_au8SlvData[slave_buff_addr]);
                slave_buff_addr++;
            }
            else            /* Own SLA+W has been receive; ACK has been return */
            {
                s_Event = SLAVE_GET_DATA;
            }
            g_u8DeviceAddr = (unsigned char)UI2C_GET_DATA(UI2C1);
        }
        else if(s_Event == SLAVE_GET_DATA)
        {
            temp = (unsigned char)UI2C_GET_DATA(UI2C1);
            g_au8RxData[g_u8DataLenS] = temp;
            g_u8DataLenS++;

            if(g_u8DataLenS == 2)   /* Address has been received; ACK has been returned*/
            {
                temp = (g_au8RxData[0] << 8);
                temp += g_au8RxData[1];
                slave_buff_addr = temp;
            }
            if(g_u8DataLenS == 3)   /* Data has been received; ACK has been returned*/
            {
                temp = g_au8RxData[2];
                g_au8SlvData[slave_buff_addr] = temp;
                g_u8DataLenS = 0;
            }
        }
        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)       /* Data in TXDAT has been transmitted; NACK has been received */
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STORIF_Msk); /* Clear STOP INT Flag */
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

    /* I2C pin enable schmitt trigger */
    PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk | GPIO_SMTEN_SMTEN2_Msk);

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

    UI2C_SetSlaveAddrMask(UI2C1, 0, 0x01);

    UI2C_ENABLE_PROT_INT(UI2C1, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI1_IRQn);
}

int main()
{
    uint32_t i;
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|          USCI_I2C Driver Sample Code(Slave)           |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init USCI_I2C1 */
    UI2C1_Init();

    /* USCI_I2C1 enter no address SLV mode */
    s_Event = SLAVE_ADDRESS_ACK;
    UI2C_SET_CONTROL_REG(UI2C1, UI2C_CTL_AA);

    for (i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* USCI_I2C1 function to Slave receive/transmit data */
    s_UI2C1HandlerFn = USCI_I2C_SlaveTRx;

    printf("\n");
    printf("USCI_I2C1 Slave Mode is Running.\n");

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
