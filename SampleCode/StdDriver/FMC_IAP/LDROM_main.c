/******************************************************************************
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 18/07/17 4:06p $
 * @brief    This sample code includes LDROM image (fmc_ld_iap)
 *           and APROM image (fmc_ap_main).
 *           It shows how to branch between APROM and LDROM. To run
 *           this sample code, the boot mode must be "Boot from APROM
 *           with IAP".
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"
#include "fmc.h"

typedef void (FUNC_PTR)(void);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable 48MHz HIRC */
    CLK->PWRCTL = CLK->PWRCTL | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for 48MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

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


#ifdef __ARMCC_VERSION
void __set_SP(uint32_t _sp)
{
		__ASM(
    "MSR MSP, r0 \n"
    "BX lr \n"			
			);
}
#endif

uint32_t UUART_Open(UUART_T* uart, uint32_t u32baudrate)
{
    int32_t multiple;

    uart->CTL = 0x02;

    multiple = __HIRC / u32baudrate;    /* Fsys_clk = 2 * _HXT*/
    multiple = multiple /5;             /* Set DSCNT = 4 --> divide by (4+1) */
    uart-> BRGEN = (multiple << 16) | (0x04 << 10);

    uart-> DATIN0 = 0x10;               /* input falling edge activate */

    uart->LINECTL = 0x08<<8 | 0x01;     /* 8-N-1 bit, LSB first */

    /* Uart Protocol setting */
    uart->PROTCTL = UUART_PROTCTL_PROTEN_Msk;

    return 0;
}
void SendChar_ToUART(int ch)
{
    while (UUART0->BUFSTS & UUART_BUFSTS_TXFULL_Msk);
    UUART0->TXDAT = ch;

    if (ch == '\n')
    {
        while (UUART0->BUFSTS & UUART_BUFSTS_TXFULL_Msk);
        UUART0->TXDAT = '\r';
    }
}


void print_msg(char *str)
{
    for ( ; *str ; str++)
        SendChar_ToUART(*str);
}

int main()
{
#ifdef __GNUC__                        /* for GNU C compiler */
    uint32_t    u32Data;
#endif
    FUNC_PTR    *func;

    SYS_Init();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* Enable FMC ISP function */
    FMC_Open();

    print_msg("\n\n");
    print_msg("Mini57 FMC IAP Sample Code [LDROM code]\n");

    print_msg("\n\nPress any key to branch to APROM...\n");

    while(UUART0->BUFSTS & UUART_BUFSTS_RXEMPTY_Msk);   /* Check RX empty => failed */

    print_msg("\n\nChange VECMAP and branch to APROM...\n");

    while((UUART0->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk) == 0);   /* Wait Tx empty */

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */
    /* FMC_SetVectorPageAddr(FMC_APROM_BASE) */
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    func = (FUNC_PTR *) FMC_Read(FMC_APROM_BASE+4);

#if defined ( __GNUC__ ) && !defined (__ARMCC_VERSION)                  /* for GNU C compiler */
    u32Data = *(uint32_t *)FMC_LDROM_BASE;
    asm("msr msp, %0" : : "r" (u32Data));
#else
    __set_SP( *( volatile uint32_t *) FMC_APROM_BASE);
#endif

    func();

    while (1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
