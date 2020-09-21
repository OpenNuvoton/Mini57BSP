/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 18/07/17 3:59p $
 * @brief    Show FMC read flash IDs, erase, read, and write functions.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"
#include "fmc.h"

#define APROM_TEST_BASE             0x3000
#define DATA_FLASH_TEST_BASE        0x3000
#define DATA_FLASH_TEST_END         0x4000
#define TEST_PATTERN                0x5A5A5A5A

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

    multiple = __HIRC / u32baudrate;            /* Fsys_clk = 2 * _HXT*/
    multiple = multiple /5;                     /* Set DSCNT = 4 --> divide by (4+1) */
    uart-> BRGEN = (multiple << 16) | (0x04 << 10);

    uart-> DATIN0 = 0x10;                       /* input falling edge activate */

    uart->LINECTL = 0x08<<8 | 0x01;             /* 8-N-1 bit, LSB first */

    /* Uart Protocol setting */
    uart->PROTCTL = UUART_PROTCTL_PROTEN_Msk;

    return 0;
}

static int  set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];

    /* FMC_ReadConfig(au32Config, 2) */
    /*   u32Config[0] = FMC_Read(FMC_CONFIG_BASE) */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    au32Config[0] = FMC->ISPDAT;

    /*   u32Config[1] = FMC_Read(FMC_CONFIG_BASE+4) */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = (FMC_CONFIG_BASE+4);
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    au32Config[1] = FMC->ISPDAT;

    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;

    /* FMC_ENABLE_CFG_UPDATE(); */
    FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk;

    au32Config[0] &= ~0x1;
    au32Config[1] = u32DFBA;

    /* FMC_WriteConfig(au32Config, 2) */
    /*   FMC_ENABLE_CFG_UPDATE() */
    FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk;
    /*   FMC_Erase(FMC_CONFIG_BASE) */
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;

    /*   FMC_Write(FMC_CONFIG_BASE, u32Config[0]) */
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPDAT = au32Config[0];
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    /*   FMC_Write(FMC_CONFIG_BASE+4, u32Config[1]) */
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = (FMC_CONFIG_BASE+4);
    FMC->ISPDAT = au32Config[1];
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    /*   FMC_DISABLE_CFG_UPDATE() */
    FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk;

    printf("\nSet Data Flash base as 0x%x.\n", DATA_FLASH_TEST_BASE);

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    return 0;
}


void run_crc32_checksum()
{
    uint32_t    chksum;
    int32_t    ret;

    /* FMC_GetCRC32Sum(FMC_APROM_BASE, DATA_FLASH_TEST_BASE-FMC_APROM_BASE, &chksum) */
    ret = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPDAT  = (DATA_FLASH_TEST_BASE-FMC_APROM_BASE);
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum0;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum0;
    }

    chksum = FMC->ISPDAT;

checksum0:
    if (ret == 0)
        printf("  APROM CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  APROM CRC32 checksum .................. ERROR!!\n");


    /* FMC_GetCRC32Sum(DATA_FLASH_TEST_BASE, FMC_APROM_END-DATA_FLASH_TEST_BASE, &chksum) */
    ret = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = DATA_FLASH_TEST_BASE;
    FMC->ISPDAT  = (FMC_APROM_END-DATA_FLASH_TEST_BASE);
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum1;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = DATA_FLASH_TEST_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum1;
    }

    chksum = FMC->ISPDAT;

checksum1:
    if (ret == 0)
        printf("  Data Flash CRC32 checksum ............. [0x%08x]\n", chksum);
    else
        printf("  Data flash CRC32 checksum ............. ERROR!!\n");

    /* FMC_GetCRC32Sum(FMC_LDROM_BASE, FMC_LDROM_SIZE, &chksum) */
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
        goto checksum2;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_LDROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum2;
    }

    chksum = FMC->ISPDAT;

checksum2:
    if (ret == 0)
        printf("  LDROM CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  LDROM CRC32 checksum .................. ERROR!!\n");

    /* FMC_GetCRC32Sum(FMC_SPROM0_BASE, FMC_SPROM_SIZE, &chksum) */
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
        goto checksum3;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_SPROM0_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum3;
    }

    chksum = FMC->ISPDAT;

checksum3:
    if (ret == 0)
        printf("  SPROM0 CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  SPROM0 CRC32 checksum .................. ERROR!!\n");


    /* FMC_GetCRC32Sum(FMC_SPROM1_BASE, FMC_SPROM_SIZE, &chksum) */
    ret = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = FMC_SPROM1_BASE;
    FMC->ISPDAT  = FMC_SPROM_SIZE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum4;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_SPROM1_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum4;
    }

    chksum = FMC->ISPDAT;

checksum4:
    if (ret == 0)
        printf("  SPROM1 CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  SPROM1 CRC32 checksum .................. ERROR!!\n");


    /* FMC_GetCRC32Sum(FMC_SPROM2_BASE, FMC_SPROM_SIZE, &chksum) */
    ret = 0;
    FMC->ISPCMD = FMC_ISPCMD_CAL_CRC32;
    FMC->ISPADDR = FMC_SPROM2_BASE;
    FMC->ISPDAT  = FMC_SPROM_SIZE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum5;
    }

    FMC->ISPCMD = FMC_ISPCMD_READ_CRC32;
    FMC->ISPADDR = FMC_SPROM2_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        ret = -1;
        goto checksum5;
    }

    chksum = FMC->ISPDAT;

checksum5:
    if (ret == 0)
        printf("  SPROM2 CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  SPROM2 CRC32 checksum .................. ERROR!!\n");
}


int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        /* FMC_Write(u32Addr, u32Pattern) */
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = u32Addr;
        FMC->ISPDAT = u32Pattern;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    }
    return 0;
}


int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        /* u32data = FMC_Read(u32Addr) */
        FMC->ISPCMD = FMC_ISPCMD_READ;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
        u32data = FMC->ISPDAT;

        if (u32data != u32Pattern)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;
        }
    }
    return 0;
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page */
        if(u32Addr >= FMC_SPROM0_BASE)
        {
            /* FMC_Erase_SPROM(u32Addr) */
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPDAT = 0x0055AA03;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }
        else
        {
            /* FMC_Erase(u32Addr) */
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        /* Write test pattern to fill the whole page */
        if (fill_data_pattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all equal to test pattern */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }

        if(u32Addr >= FMC_SPROM0_BASE)
        {
            /* FMC_Erase_SPROM(u32Addr) */
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPDAT = 0x0055AA03;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }
        else
        {
            /* FMC_Erase(u32Addr) */
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }
    }
    printf("\r    Flash Test Passed.          \n");
    return 0;
}


int main()
{
    uint32_t    i, u32Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          Mini57 FMC Sample Code        |\n");
    printf("+----------------------------------------+\n");

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    /* FMC_Open() */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    if (set_data_flash_base(DATA_FLASH_TEST_BASE) < 0)
    {
        printf("Failed to set Data Flash base address!\n");
        goto lexit;
    }


    /* Read BS */
    printf("  Boot Mode ............................. ");
    /* if (FMC_GetBootSource() == 0) */
    if((FMC->ISPCTL & FMC_ISPCTL_BS_Msk) == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    /* u32Data = FMC_ReadCID()  */
    FMC->ISPCMD = FMC_ISPCMD_READ_CID;
    FMC->ISPADDR = 0x0;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* u32Data = FMC_ReadPID()  */
    FMC->ISPCMD = FMC_ISPCMD_READ_PID;
    FMC->ISPADDR = 0x04;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for (i = 0; i < 3; i++)
    {
        /* u32Data = FMC_ReadUID(i) */
        FMC->ISPCMD = FMC_ISPCMD_READ_UID;
        FMC->ISPADDR = 0x04 * i;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
        u32Data = FMC->ISPDAT;

        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);
    }

    for (i = 0; i < 4; i++)
    {
        /* u32Data = FMC_ReadUCID(i) */
        FMC->ISPCMD = FMC_ISPCMD_READ_UID;
        FMC->ISPADDR = (0x04 * i) + 0x10;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
        u32Data = FMC->ISPDAT;

        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);
    }

    /* Read User Configuration */
    /* printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE)) */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;
    printf("  User Config 0 ......................... [0x%08x]\n", u32Data);

    /* printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4)) */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = (FMC_CONFIG_BASE+4);
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);
    u32Data = FMC->ISPDAT;
    printf("  User Config 1 ......................... [0x%08x]\n", u32Data);

    /* Read Data Flash base address */
    /* u32Data = FMC_ReadDataFlashBaseAddr() */
    u32Data = FMC->DFBA;

    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);

    run_crc32_checksum();

    printf("\n\nLDROM test =>\n");
    /* FMC_ENABLE_LD_UPDATE() */
    FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk;

    if (flash_test(FMC_LDROM_BASE, FMC_LDROM_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");
        goto lexit;
    }
    /* FMC_DISABLE_LD_UPDATE() */
    FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;

    printf("\n\nSPROM test =>\n");
    /* FMC_ENABLE_SP_UPDATE() */
    FMC->ISPCTL |=  FMC_ISPCTL_SPUEN_Msk;

    if (flash_test(FMC_SPROM0_BASE, FMC_SPROM0_END - 4, TEST_PATTERN) < 0)
    {
        printf("\n\nSPROM test failed!\n");
        goto lexit;
    }
    /* FMC_DISABLE_SP_UPDATE() */
    FMC->ISPCTL &= ~FMC_ISPCTL_SPUEN_Msk;

    printf("\n\nAPROM test =>\n");
    /* FMC_ENABLE_AP_UPDATE() */
    FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk;

    if (flash_test(APROM_TEST_BASE, DATA_FLASH_TEST_BASE, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");
        goto lexit;
    }
    /* FMC_DISABLE_AP_UPDATE() */
    FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk;

    printf("\n\nData Flash test =>\n");
    if (flash_test(DATA_FLASH_TEST_BASE, DATA_FLASH_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nData Flash test failed!\n");
        goto lexit;
    }

    printf("\n\nChecksum test =>\n");
    run_crc32_checksum();

lexit:

    /* Disable FMC ISP function */
    /* FMC_Close() */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");
    while (1);

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
