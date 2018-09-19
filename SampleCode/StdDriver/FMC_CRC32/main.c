/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 18/07/17 4:42p $
 * @brief    Show FMC CRC32 calculating capability.
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "Mini57Series.h"
#include "fmc.h"

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

    FMC_Open();                        /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0xDA. */
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Read product ID. */
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));

    /* Read Data Flash base address */
    printf("  Data Flash Base Address ............... [0x%08x]\n", FMC_ReadDataFlashBaseAddr());

    printf("\nLDROM (0x100000 ~ 0x101200) CRC32 checksum =>  ");

    /* 
     *  Request FMC hardware to run CRC32 calculation on flash range from FMC_LDROM_BASE and
     *  length is FMC_LDROM_SIZE. The CRC32 calculation result will be put in u32ChkSum.
     */
    ret = FMC_GetCRC32Sum(FMC_LDROM_BASE, FMC_LDROM_SIZE, &u32ChkSum);
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
    ret = FMC_GetCRC32Sum(FMC_SPROM0_BASE, FMC_SPROM_SIZE, &u32ChkSum);
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
    ret = FMC_GetCRC32Sum(FMC_APROM_BASE, FMC_APROM_END - FMC_APROM_BASE, &u32ChkSum);
    if (ret < 0)
    {
        printf("Failed on calculating APROM CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    printf("\nFMC CRC32 checksum test done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
