/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 18/07/17 3:49p $
 * @brief    Show FMC CRC32 calculating capability.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "Mini57Series.h"
#include "fmc.h"

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

    /* Enable USCI0 IP clock */
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_USCI0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

uint32_t UUART_Open(UUART_T* uart, uint32_t u32baudrate)
{
    int32_t multiple;

    uart->CTL = 0x02;

    multiple = __HIRC / u32baudrate;       /* Fsys_clk = 2 * _HXT*/
    multiple = multiple /5;                /* Set DSCNT = 4 --> divide by (4+1) */
    uart-> BRGEN = (multiple << 16) | (0x04 << 10);

    uart-> DATIN0 = 0x10;                  /* input falling edge activate */

    uart->LINECTL = 0x08<<8 | 0x01;        /* 8-N-1 bit, LSB first */

    /* Uart Protocol setting */
    uart->PROTCTL = UUART_PROTCTL_PROTEN_Msk;

    return 0;
}

int main()
{
    int         ret;                   /* return value */
    uint32_t    u32Data, u32ChkSum;    /* temporary data */

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("+--------------------------------------------+\n");
    printf("|        Mini57 FMC_CRC32 Sample Code        |\n");
    printf("+--------------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers */

    /* Enable FMC ISP function */
    /* FMC_Open() */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    /* Read company ID. Should be 0xDA. */
    /* u32Data = FMC_ReadCID()  */
    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADDR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read product ID. */
    /* u32Data = FMC_ReadPID()  */
    FMC->ISPCMD = FMC_ISPCMD_READ_PID;
    FMC->ISPADDR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;

    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    /* printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE)) */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;
    printf("  User Config 0 ......................... [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG1 */
    /* printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4)) */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = (FMC_CONFIG_BASE+4);
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;

    printf("  User Config 1 ......................... [0x%08x]\n", u32Data);

    /* Read Data Flash base address */
    /* printf("  Data Flash Base Address ............... [0x%08x]\n", FMC_ReadDataFlashBaseAddr()) */
    u32Data = FMC->DFBA;
    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);

    printf("\nLDROM (0x100000 ~ 0x101200) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on flash range from FMC_LDROM_BASE and
     *  length is FMC_LDROM_SIZE. The CRC32 calculation result will be put in u32ChkSum.
     */
    /* ret = FMC_GetCRC32Sum(FMC_LDROM_BASE, FMC_LDROM_SIZE, &u32ChkSum) */
    ret = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = FMC_LDROM_BASE;
    FMC->ISPDAT  = FMC_LDROM_SIZE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum0;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_LDROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum0;
    }

    u32ChkSum = FMC->ISPDAT;

checksum0:
    if (ret < 0)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }
    printf("0x%x\n", u32ChkSum);       /* print out LDROM CRC32 check sum value */

    printf("\nSPROM (0x200000 ~ 0x200200) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on flash range from FMC_SPROM_BASE and
     *  length is FMC_SPROM_SIZE. The CRC32 calculation result will be put in u32ChkSum.
     */
    /* ret = FMC_GetCRC32Sum(FMC_SPROM0_BASE, FMC_SPROM_SIZE, &u32ChkSum) */
    ret = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = FMC_SPROM0_BASE;
    FMC->ISPDAT  = FMC_SPROM_SIZE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum1;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_SPROM0_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum1;
    }

    u32ChkSum = FMC->ISPDAT;

checksum1:

    if (ret < 0)
    {
        printf("Failed on calculating SPROM CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out SPROM CRC32 check sum value */

    printf("\nAPROM (0x0 ~ 0x75FF) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on flash range from FMC_APROM_BASE and
     *  length is FMC_APROM_END. The CRC32 calculation result will be put in u32ChkSum.
     */
    /* ret = FMC_GetCRC32Sum(FMC_APROM_BASE, FMC_APROM_END - FMC_APROM_BASE, &u32ChkSum) */
    ret = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPDAT  = (FMC_APROM_END - FMC_APROM_BASE);
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum2;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum2;
    }

    u32ChkSum = FMC->ISPDAT;

checksum2:
    if (ret < 0)
    {
        printf("Failed on calculating APROM CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    printf("\nFMC CRC32 checksum test done.\n");

lexit:
    /* Disable FMC ISP function */
    /* FMC_Close(); */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
