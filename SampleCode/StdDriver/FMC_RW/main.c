/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 18/07/17 4:44p $
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
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}


static int  set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];

    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;

    FMC_ENABLE_CFG_UPDATE();

    au32Config[0] &= ~0x1;
    au32Config[1] = u32DFBA;

    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;

    printf("\nSet Data Flash base as 0x%x.\n", DATA_FLASH_TEST_BASE);

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    return 0;
}


void run_crc32_checksum()
{
    uint32_t    chksum;

    if (FMC_GetCRC32Sum(FMC_APROM_BASE, DATA_FLASH_TEST_BASE-FMC_APROM_BASE, &chksum) == 0)
        printf("  APROM CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  APROM CRC32 checksum .................. ERROR!!\n");

    if (FMC_GetCRC32Sum(DATA_FLASH_TEST_BASE, FMC_APROM_END-DATA_FLASH_TEST_BASE, &chksum) == 0)
        printf("  Data Flash CRC32 checksum ............. [0x%08x]\n", chksum);
    else
        printf("  Data flash CRC32 checksum ............. ERROR!!\n");

    if (FMC_GetCRC32Sum(FMC_LDROM_BASE, FMC_LDROM_SIZE, &chksum) == 0)
        printf("  LDROM CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  LDROM CRC32 checksum .................. ERROR!!\n");

    if (FMC_GetCRC32Sum(FMC_SPROM0_BASE, FMC_SPROM_SIZE, &chksum) == 0)
        printf("  SPROM0 CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  SPROM0 CRC32 checksum .................. ERROR!!\n");

    if (FMC_GetCRC32Sum(FMC_SPROM1_BASE, FMC_SPROM_SIZE, &chksum) == 0)
        printf("  SPROM1 CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  SPROM1 CRC32 checksum .................. ERROR!!\n");

    if (FMC_GetCRC32Sum(FMC_SPROM2_BASE, FMC_SPROM_SIZE, &chksum) == 0)
        printf("  SPROM2 CRC32 checksum .................. [0x%08x]\n", chksum);
    else
        printf("  SPROM2 CRC32 checksum .................. ERROR!!\n");
}


int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC_Write(u32Addr, u32Pattern);
    }
    return 0;
}


int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32data = FMC_Read(u32Addr);
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
            FMC_Erase_SPROM(u32Addr);
        else
            FMC_Erase(u32Addr);

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
            FMC_Erase_SPROM(u32Addr);
        else
            FMC_Erase(u32Addr);

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
    FMC_Open();

    if (set_data_flash_base(DATA_FLASH_TEST_BASE) < 0)
    {
        printf("Failed to set Data Flash base address!\n");
        goto lexit;
    }

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    u32Data = FMC_ReadCID();
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for (i = 0; i < 3; i++)
    {
        u32Data = FMC_ReadUID(i);
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);
    }

    for (i = 0; i < 4; i++)
    {
        u32Data = FMC_ReadUCID(i);
        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);
    }

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));

    /* Read Data Flash base address */
    u32Data = FMC_ReadDataFlashBaseAddr();
    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);

    run_crc32_checksum();

    printf("\n\nLDROM test =>\n");
    FMC_ENABLE_LD_UPDATE();
    if (flash_test(FMC_LDROM_BASE, FMC_LDROM_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");
        goto lexit;
    }
    FMC_DISABLE_LD_UPDATE();

    printf("\n\nSPROM test =>\n");
    FMC_ENABLE_SP_UPDATE();
    if (flash_test(FMC_SPROM0_BASE, FMC_SPROM0_END - 4, TEST_PATTERN) < 0)
    {
        printf("\n\nSPROM test failed!\n");
        goto lexit;
    }
    FMC_DISABLE_SP_UPDATE();

    printf("\n\nAPROM test =>\n");
    FMC_ENABLE_AP_UPDATE();
    if (flash_test(APROM_TEST_BASE, DATA_FLASH_TEST_BASE, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");
        goto lexit;
    }
    FMC_DISABLE_AP_UPDATE();

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
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");
    while (1);

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
