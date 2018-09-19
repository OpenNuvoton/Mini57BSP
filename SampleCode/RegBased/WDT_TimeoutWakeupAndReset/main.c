/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/08/07 4:33p $
 * @brief    Implement WDT time-out interrupt event to wake up system and generate time-out reset system event while WDT time-out reset delay period expired.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32WDTINTCounts;
volatile uint8_t g_u8IsWDTWakeupINT;

/**
 * @brief       IRQ Handler for WDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT, declared in startup_Mini57Series.s.
 */
void WDT_IRQHandler(void)
{
    if(g_u32WDTINTCounts < 10)
        WDT->CTL  = (WDT->CTL & ~(WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk | WDT_CTL_RSTF_Msk)) | WDT_CTL_RSTCNT_Msk;

    if(WDT->CTL & WDT_CTL_IF_Msk)
    {
        /* Clear WDT time-out interrupt flag */
        WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_IF_Msk;

        g_u32WDTINTCounts++;
    }

    if(WDT->CTL & WDT_CTL_WKF_Msk)
    {
        /* Clear WDT time-out wake-up flag */
        WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk)) | WDT_CTL_WKF_Msk;

        g_u8IsWDTWakeupINT = 1;
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
    CLK->APBCLK = CLK->APBCLK | (CLK_APBCLK_USCI0CKEN_Msk | CLK_APBCLK_WDTCKEN_Msk);

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK->CLKSEL1 | CLK_WDT_SRC_LIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

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

int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    WDT Time-out Wake-up Sample Code    |\n");
    printf("+----------------------------------------+\n\n");
    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT->CTL & WDT_CTL_RSTF_Msk)
    {
        WDT->CTL = (WDT->CTL & ~(WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_RSTF_Msk;
        printf("*** System has been reset by WDT time-out event ***\n\n");
        while(1);
    }

    printf("# WDT Settings:\n");
    printf("    - Clock source is LIRC                  \n");
    printf("    - Time-out interval is 2^14 * WDT clock \n");
    printf("      (around 1.6384 ~ 1.7408 s)            \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("    - Reset function enable                 \n");
    printf("# System will generate a WDT time-out interrupt event after 1.6384 ~ 1.7408 s, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use PA.0 high/low period time to check WDT time-out interval)\n");
    printf("# When WDT interrupt counts large than 10, \n");
    printf("    we will not reset WDT counter value and system will be reset immediately by WDT time-out reset signal.\n");

    /* Use PA.0 to check WWDT compare match interrupt period time */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
    PA0 = 1;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Configure WDT settings and start WDT counting */
    g_u32WDTINTCounts = g_u8IsWDTWakeupINT = 0;

    WDT->CTL = WDT_TIMEOUT_2POW14 | WDT_CTL_WDTEN_Msk |
               (1 << WDT_CTL_RSTEN_Pos) |
               (1 << WDT_CTL_WKEN_Pos);

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Enable WDT interrupt function */
    WDT->CTL |= WDT_CTL_INTEN_Msk;

    while(1)
    {
        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        printf("\nSystem enter to power-down mode ...\n");
        /* Wait USCI UART buffer empty to get a cleaner console out */
        UUART0->PROTSTS = (UUART_PROTSTS_TXENDIF_Msk | UUART_PROTSTS_TXSTIF_Msk);
        while(!((UUART0->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk) >> UUART_BUFSTS_TXEMPTY_Pos));
        if(UUART0->PROTSTS & UUART_PROTSTS_TXSTIF_Msk)
            while((UUART0->PROTSTS & UUART_PROTSTS_TXENDIF_Msk) != UUART_PROTSTS_TXENDIF_Msk);

        /* Enable PD.0 (nRESET pin) interrupt that trigger by falling edge to make sure
           RESET button can wake up system from power down mode. */
        PD->INTTYPE &= (~BIT0);     /* edge trigger for PD0 */
        PD->INTEN   |=   BIT0;      /* enable falling or low trigger for PD0 */

        /* enable M0 register SCR[SEVONPEND] and SCR[SLEEPDEEP] */
        SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SEVONPEND_Msk);
        /* clear interrupt status and enable wake up interrupt */
        CLK->PWRCTL |= (CLK_PWRCTL_PDWKIF_Msk | CLK_PWRCTL_PDWKIEN_Msk);
        /* enable system power-down feature */
        CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);
        /* execute Wait For Interrupt; Enter power-down mode since CLK_PWRCTL_PDEN_Msk is 1 */
        __WFI();

        /* Check if WDT time-out interrupt and wake-up occurred or not */
        while(g_u8IsWDTWakeupINT == 0);

        g_u8IsWDTWakeupINT = 0;
        PA0 ^= 1;

        printf("System has been waken up done. WDT interrupt counts: %d.\n\n", g_u32WDTINTCounts);
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
