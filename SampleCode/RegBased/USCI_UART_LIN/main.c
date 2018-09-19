/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/30 7:42p $
 * @brief    Transmit LIN frame including header and response in USCI UART LIN mode.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


/* CheckSum Method */
#define MODE_CLASSIC    2
#define MODE_ENHANCED   1


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t g_i32pointer;
uint8_t g_u8SendData[12] ;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTest(void);
void LIN_MasterTest(uint32_t u32id, uint32_t u32ModeSel);
void LIN_SendHeader(uint32_t u32id);
void LIN_SendResponse(int32_t checkSumOption, uint32_t *pu32TxBuf);
void LIN_SendResponseWithByteCnt(int32_t checkSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt);
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel);
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt);


/*---------------------------------------------------------------------------------------------------------*/
/*  Sample Code Menu                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_TestItem()
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     LIN Master Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Master send header with ID = 0x30                     |\n");
    printf("| [2] Master send header and response with classic checksum |\n");
    printf("| [3] Master send header and response with enhanced checksum|\n");
    printf("|                                                           |\n");
    printf("| To measure UART1_TXD(PD.3) to check waveform ...          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTest()
{
    uint32_t u32Item;

    /* Set UUART Configuration, LIN Max Speed is 20K */
    UUART1->BRGEN = (624 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos); /* Set UART baud rate as 9600bps */

    /* Set USCI_UART line configuration */
    UUART1->LINECTL = (UUART1->LINECTL & ~UUART_LINECTL_DWIDTH_Msk) | UUART_WORD_LEN_8;
    UUART1->PROTCTL = (UUART1->PROTCTL & ~(UUART_PROTCTL_STICKEN_Msk | UUART_PROTCTL_EVENPARITY_Msk |
                                           UUART_PROTCTL_PARITYEN_Msk)) | UUART_PARITY_NONE;
    UUART1->PROTCTL = (UUART1->PROTCTL & ~UUART_PROTCTL_STOPB_Msk ) | UUART_STOP_BIT_1;

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UUART1 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UUART1 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UUART1 Tx pin to check it.
    */

    do
    {
        LIN_TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);
        switch(u32Item)
        {
        case '1':
            LIN_SendHeader(0x30);
            break;
        case '2':
            LIN_MasterTest(0x35, MODE_CLASSIC);
            break;
        case '3':
            LIN_MasterTest(0x12, MODE_ENHANCED);
            break;
        default:
            break;
        }
    }
    while(u32Item != 27);

    UUART1->CTL = 0;

    printf("\nLIN Sample Code End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Master send header and response                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTest(uint32_t u32id, uint32_t u32ModeSel)
{
    uint32_t testPattern[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};

    /* Send ID=0x35 Header and Response TestPatten */
    LIN_SendHeader(u32id);
    LIN_SendResponse(u32ModeSel, &testPattern[0]);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute Parity Value                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t GetParityValue(uint32_t u32id)
{
    uint32_t u32Res = 0, ID[6], p_Bit[2] , mask = 0;

    for(mask = 0; mask < 6; mask++)
        ID[mask] = (u32id & (1 << mask)) >> mask;

    p_Bit[0] = (ID[0] + ID[1] + ID[2] + ID[4]) % 2;
    p_Bit[1] = (!((ID[1] + ID[3] + ID[4] + ID[5]) % 2));

    u32Res = u32id + (p_Bit[0] << 6) + (p_Bit[1] << 7);
    return u32Res;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value , MODE_CLASSIC:(Not Include ID)    MODE_ENHANCED:(Include ID)                    */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel)
{
    uint32_t i, CheckSum = 0;

    for(i = u32ModeSel; i <= 9; i++)
    {
        CheckSum += pu8Buf[i];
        if(CheckSum >= 256)
            CheckSum -= 255;
    }
    return (255 - CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt)
{
    uint32_t i, CheckSum = 0;

    for(i = 0 ; i < u32ByteCnt; i++)
    {
        CheckSum += pu8Buf[i];
        if(CheckSum >= 256)
            CheckSum -= 255;
    }
    return (uint8_t)(255 - CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Header Field                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendHeader(uint32_t u32id)
{
    uint32_t u32Count;
    g_i32pointer = 0 ;

    /* Set LIN operation mode, Tx mode and break field length is 12 bits */
    UUART1->PROTCTL &= ~(UUART_PROTCTL_LINRXEN_Msk | UUART_PROTCTL_LINBRKEN_Msk);
    UUART1->PROTCTL |= UUART_PROTCTL_LINBRKEN_Msk;

    g_u8SendData[g_i32pointer++] = 0x55 ;                   // SYNC Field
    g_u8SendData[g_i32pointer++] = GetParityValue(u32id);   // ID+Parity Field

    for(u32Count = 0; u32Count < 2; u32Count++)
    {
        while(!(UUART1->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk));   /* Wait Tx empty */

        UUART1->TXDAT = g_u8SendData[u32Count];
    }
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponse(int32_t checkSumOption, uint32_t *pu32TxBuf)
{
    int32_t i32;

    for(i32 = 0; i32 < 8; i32++)
        g_u8SendData[g_i32pointer++] = pu32TxBuf[i32] ;

    g_u8SendData[g_i32pointer++] = GetCheckSumValue(g_u8SendData, checkSumOption) ; //CheckSum Field

    for(i32 = 0; i32 < 9; i32++)
    {
        while(!(UUART1->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk));   /* Wait Tx empty */

        UUART1->TXDAT = g_u8SendData[i32 + 2];
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponseWithByteCnt(int32_t checkSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt)
{
    int32_t i32;

    /* Prepare data */
    for(i32 = 0; i32 < u32ByteCnt; i32++)
        g_u8SendData[g_i32pointer++] = pu32TxBuf[i32] ;

    /* Prepare check sum */
    if(checkSumOption == MODE_CLASSIC)
        g_u8SendData[g_i32pointer++] = GetCheckSumValue(&g_u8SendData[2], u32ByteCnt) ;  //CheckSum Field
    else if(checkSumOption == MODE_ENHANCED)
        g_u8SendData[g_i32pointer++] = GetCheckSumValue(&g_u8SendData[1], (u32ByteCnt + 1)) ; //CheckSum Field

    /* Send data and check sum */
    for(i32 = 0; i32 < 9; i32++)
    {
        while(!(UUART1->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk));   /* Wait Tx empty */

        UUART1->TXDAT = g_u8SendData[i32 + 2];
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
    /* Init USCI UART1 for testing */
    UUART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART sample LIN function */
    LIN_FunctionTest();

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
