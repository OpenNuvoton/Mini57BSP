/**************************************************************************//**
 * @file     i2c_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    I2C ISP slave sample file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t i2c_rcvbuf[64];
volatile uint8_t bI2cDataReady;
volatile uint8_t bI2cSlvEndFlag;

volatile uint8_t g_u8DeviceAddr = 0x60;
volatile uint8_t g_u8SlvDataLen;

volatile uint16_t g_u16RecvAddr;
volatile enum UI2C_SLAVE_EVENT s_Event;


void UI2C_Init(void)
{
    uint32_t u32ClkDiv;
    //uint32_t u32ClkSpeed;
    //uint32_t u32Pclk = SystemCoreClock;

    /* The USCI usage is exclusive */
    /* If user configure the USCI port as UI2C function, that port cannot use UUART or USPI function. */
    /* Open USCI_I2C0 and set clock to 100k */
    //u32ClkSpeed = 100000;
    //u32ClkDiv = (uint32_t) ((((((u32Pclk/2)*10)/(u32ClkSpeed))+5)/10)-1); /* Compute proper divider for USCI_I2C clock */
    u32ClkDiv = 0xEF;

    /* Enable USCI_I2C protocol */
    UI2C0->CTL = (4 << UI2C_CTL_FUNMODE_Pos);

    /* Data format configuration */
    /* 8 bit data length */
    UI2C0->LINECTL = (8 << UI2C_LINECTL_DWIDTH_Pos);

    /* MSB data format */
    //UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    /* Set USCI_I2C bus clock to 100k */
    UI2C0->BRGEN = (UI2C0->BRGEN & ~UI2C_BRGEN_CLKDIV_Msk) | (u32ClkDiv << UI2C_BRGEN_CLKDIV_Pos);

    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;

    /* Set USCI_I2C0 Slave Addresses */
    UI2C0->DEVADDR0  = g_u8DeviceAddr;    /* Slave Address : 0x60 */

    s_Event = SLAVE_ADDRESS_ACK;
    bI2cSlvEndFlag = 0;

    /* UI2C enter no address SLV mode */
    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    //UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    //NVIC_EnableIRQ(USCI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 IRQ Handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (UI2C_GET_TIMEOUT_FLAG(UI2C0))
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C0->PROTSTS = UI2C_PROTSTS_TOIF_Msk;
    }
    else
    {
        UI2C_SlaveTRx(UI2C0, u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UI2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SlaveTRx(UI2C_T *ui2c, uint32_t u32Status)
{
    uint8_t u8data;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)                   /* Re-Start been received */
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        s_Event = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)                /* USCI I2C Bus have been received ACK */
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (s_Event == SLAVE_ADDRESS_ACK)                                                   /* Address Data has been received */
        {

            /* USCI I2C receives Slave command type */
            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                g_u8SlvDataLen = 0;

                u8data = response_buff[g_u8SlvDataLen];

                UI2C_SET_DATA(ui2c, u8data);
                g_u8SlvDataLen++;

                s_Event = SLAVE_SEND_DATA;                                                  /* Slave address read has been received */
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
            }
            else
            {
                bI2cDataReady = 0;
                bISPDataReady = 0;
                g_u8SlvDataLen = 0;

                s_Event = SLAVE_GET_DATA;       /* Slave address write has been received */
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
            }

            /* Read address from USCI I2C RXDAT*/
            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_GET_DATA)     /* Previously address with own SLA address Data has been received; ACK has been returned*/
        {
            i2c_rcvbuf[g_u8SlvDataLen] = UI2C_GET_DATA(ui2c);
            g_u8SlvDataLen++;
            g_u8SlvDataLen &= 0x3F;
            bI2cDataReady = (g_u8SlvDataLen == 0);

            if (g_u8SlvDataLen == 0x3F)
            {
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG));
            }
            else
            {
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
            }
        }
        else if (s_Event == SLAVE_SEND_DATA)
        {
            u8data = response_buff[g_u8SlvDataLen];

            UI2C_SET_DATA(ui2c, u8data);
            g_u8SlvDataLen++;
            g_u8SlvDataLen &= 0x3F;

            if (g_u8SlvDataLen == 0x00)
            {
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG));
            }
            else
            {
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
            }
        }

    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        if (s_Event == SLAVE_GET_DATA)  /* Previously addressed with own SLA address; NOT ACK has been returned */
        {
            i2c_rcvbuf[g_u8SlvDataLen] = UI2C_GET_DATA(ui2c);
            g_u8SlvDataLen++;
            bI2cDataReady = (g_u8SlvDataLen == 64);
        }

        /* Event process */
        g_u8SlvDataLen = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        bI2cSlvEndFlag = 1;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {

        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);

        g_u8SlvDataLen = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        bI2cSlvEndFlag = 1;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));


    }
}


