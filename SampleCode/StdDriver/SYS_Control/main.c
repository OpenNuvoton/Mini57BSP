/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Sample code for System Control.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"

#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x20000FFC      /* last 4 bytes on end of SRAM */


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
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) |
                   (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Lock protected registers */
    SYS_LockReg();
}


__IO uint32_t u32PWDU_WakeFlag = 0;
__IO uint32_t u32WDT_Ticks = 0;
/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    SYS_CLEAR_BOD_INT_FLAG(); /* Clear BOD Interrupt Flag */
    printf("Brown Out is Detected\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Watchdog IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{
    /* Clear WDT interrupt flag */
    WDT_CLEAR_TIMEOUT_INT_FLAG();
    u32WDT_Ticks++;
    printf("WDT interrupt !!\n");

    /* Check WDT wake up flag */
    if(WDT_GET_TIMEOUT_WAKEUP_FLAG())
    {
        printf("Wake up by WDT\n");
        /* Clear WDT wake up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Power Down Wake Up IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    printf("PDWU_IRQHandler running...\n");
    u32PWDU_WakeFlag = 1;
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;    /* Clear Wake-up Interrupts */
}


int main()
{
    uint32_t u32data;

    /* In end of main function, program issued CPU reset and write-protection will be disabled. */
    if(SYS->REGLCTL == 1)
        SYS_LockReg();  /* enable write-protection function if it is disabled. */

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /*--- 1. Read PDID */
    printf("\n\nPDID 0x%08X\n", SYS_ReadPDID());    /* Display PDID */
    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will show some function about system manager controller and clock controller:
     * 1. Read PDID
     * 2. Get and clear reset source
     * 3. Setting about BOD
     * 4. System write-protection enable (lock) / disable (unlock)
     * 5. Power-down mode sleep and wake up by Watchdog timer
     * 6. CPU reset
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 System Control Sample Code       |\n");
    printf("+-----------------------------------------+\n");

    /*--- 6-2. CPU reset */
    if (M32(FLAG_ADDR) == SIGNATURE)
    {
        printf("  CPU Reset success!\n");   /* SRAM data still be kept if just CPU reset. */
        M32(FLAG_ADDR) = 0;
        printf("  Press any key to continue ...\n");
        getchar();
    }

    /*--- 2. Get and clear reset source */
    /* Get reset source from last operation */
    u32data = SYS_GetResetSrc();
    printf("Get Reset Source = 0x%x\n", u32data);

    /* Clear reset source */
    printf("Clear Reset Source.\n");
    SYS_ClearResetSrc(u32data);

    u32data = SYS_GetResetSrc();
    printf("Get Reset Source again = 0x%x\n\n", u32data);

    /*--- 4. System write-protection enable (lock) / disable (unlock) */
    /* Unlock protected registers for Brown-Out Detector and power down settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before BOD setting and CPU Reset */
    if (! SYS_IsRegLocked())
    {
        printf("Protected Address is Unlocked\n\n");
    }

    /*--- 3. Setting about BOD */
    /* Enable Brown-Out Detector and Low Voltage Reset function, and set Brown-Out Detector voltage 2.4V */
    SYS_EnableBOD(SYS_BODCTL_BOD_RST_EN, SYS_BODCTL_BOD_VL_2_4V);
    NVIC_EnableIRQ(BOD_IRQn);  /* Enable BOD IRQ */

    /*--- 5. Power-down mode sleep and wake up by Watchdog timer */
    /* Waiting for message send out */
    UUART_WAIT_TX_EMPTY(UUART0);
    CLK_EnableModuleClock(WDT_MODULE);  /* Enable WDT clock */
    /* WDT timeout every 2^16 WDT clock, disable system reset, enable wake up system */
    WDT_Open(WDT_TIMEOUT_2POW16, 0, FALSE, TRUE);
    WDT_EnableInt();                                /* Enable WDT interrupt */
    NVIC_EnableIRQ(WDT_IRQn);                       /* Enable WDT NVIC */

    /* Enable wake up interrupt source */
    CLK->PWRCTL |= CLK_PWRCTL_WAKEINT_EN;
    /* Enable IRQ request for PDWU interrupt */
    NVIC_EnableIRQ(PWRWU_IRQn);

    printf("u32PWDU_WakeFlag = %x\n",u32PWDU_WakeFlag);
    printf("Enter Power Down Mode and wait WDT interrupt >>>>>>>>>>>\n");
    /* clear software semaphore */
    u32PWDU_WakeFlag = 0;
    /* Waiting for message send out */
    UUART_WAIT_TX_EMPTY(UUART0);

    /* Let system enter to Power-down mode */
    CLK_PowerDown();

    printf("Waits for 3 times WDT interrupts.....\n");
    while (u32WDT_Ticks <= 3);

    printf("<<<<<<<<<< Program resumes execution.\n");
    printf("u32PWDU_WakeFlag = %x\n\n",u32PWDU_WakeFlag);

    /*--- 6-1. CPU reset */
    /* Write a signature work to SRAM to check if it is reset by software */
    M32(FLAG_ADDR) = SIGNATURE;
    printf("\n\n  >>> Reset CPU <<<\n");

    /* Reset CPU */
    SYS_ResetCPU();
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
