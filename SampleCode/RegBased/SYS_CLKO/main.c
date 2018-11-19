/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:47p $
 * @brief    Sample code for System clock output to CLKO pin.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_HCLK_SRC_HIRC;
    CLK->CLKDIV  = (CLK->CLKDIV  & ~CLK_CLKDIV_HCLKDIV_Msk)  | CLK_CLKDIV_HCLK(1);

    /* Enable USCI0 IP clock */
    CLK->APBCLK = CLK->APBCLK | CLK_APBCLK_USCI0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) |
                   (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);
    PD->MODE = (PB->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) |
               (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos) | (GPIO_MODE_INPUT << GPIO_MODE_MODE6_Pos);

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


void _CLK_SysTickDelay(uint32_t us)
{
    uint32_t delay_tick;

    delay_tick = us * CyclesPerUs;
    if (delay_tick > SysTick_LOAD_RELOAD_Msk)   /* SysTick_LOAD_RELOAD_Msk is 24 bits for Mini57 */
    {
        printf("ERROR: _CLK_SysTickDelay(): the delay tick (%d) cannot > %d !\n", us, SysTick_LOAD_RELOAD_Msk/CyclesPerUs);
        return;
    }
    SysTick->LOAD = delay_tick;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0;
}


void delay2s()
{
    _CLK_SysTickDelay(300000);  /* delay 300ms */
    _CLK_SysTickDelay(300000);  /* delay 300ms */
    _CLK_SysTickDelay(300000);  /* delay 300ms */
    _CLK_SysTickDelay(300000);  /* delay 300ms */
    _CLK_SysTickDelay(300000);  /* delay 300ms */
    _CLK_SysTickDelay(300000);  /* delay 300ms */
    _CLK_SysTickDelay(200000);  /* delay 200ms */
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will demonstrate how to output different clocks one after another
     * to the same CLKO (PA0) pin.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 System Clock Output Sample Code  |\n");
    printf("+-----------------------------------------+\n");

    printf("Set GPA0 pin as CLKO output clock.\n");
    SYS->GPA_MFP = (SYS->GPA_MFP & ~SYS_GPA_MFP_PA0MFP_Msk) | SYS_GPA_MFP_PA0_CLKO;             /* GPA0 CLKO mode */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);   /* GPA0 output mode */

    CLK->APBCLK |= CLK_APBCLK_CLKOCKEN_Msk;

    printf("CLKO = HCLK / 1 = %dHz.\n", SystemCoreClock);
    /* Set CLKO clock source and divider */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_CLKOSEL_Msk)) |
                   (CLK_CLKO_SRC_HCLK);
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk)) |
                   ((0) << CLK_CLKOCTL_FREQSEL_Pos) |
                   ((1) << CLK_CLKOCTL_DIV1EN_Pos) |
                   (CLK_CLKOCTL_CLKOEN_Msk);

    delay2s();

    printf("CLKO = HCLK / 2^(0+1) = %dHz.\n", SystemCoreClock/2);
    /* Set CLKO clock source and divider */
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_CLKOSEL_Msk)) |
                   (CLK_CLKO_SRC_HCLK);
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk)) |
                   ((0) << CLK_CLKOCTL_FREQSEL_Pos) |
                   ((0) << CLK_CLKOCTL_DIV1EN_Pos) |
                   (CLK_CLKOCTL_CLKOEN_Msk);

    delay2s();

    printf("CLKO = HCLK / 2^(2+1) = %dHz.\n", SystemCoreClock/8);
    /* Set CLKO clock source and divider */
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_CLKOSEL_Msk)) |
                   (CLK_CLKO_SRC_HCLK);
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk)) |
                   ((2) << CLK_CLKOCTL_FREQSEL_Pos) |
                   ((0) << CLK_CLKOCTL_DIV1EN_Pos) |
                   (CLK_CLKOCTL_CLKOEN_Msk);

    delay2s();

    SYS_UnlockReg();

    printf("CLKO = HXT / 1 = %dHz.\n", __HXT);
    CLK->PWRCTL |= CLK_PWRCTL_HXT_EN;
    /* Set CLKO clock source and divider */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_CLKOSEL_Msk)) |
                   (CLK_CLKO_SRC_EXT);
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk)) |
                   ((0) << CLK_CLKOCTL_FREQSEL_Pos) |
                   ((1) << CLK_CLKOCTL_DIV1EN_Pos) |
                   (CLK_CLKOCTL_CLKOEN_Msk);

    delay2s();
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN;

    printf("CLKO = HXT / 2^(1+1) = %dHz.\n", __HXT/4);
    CLK->PWRCTL |= CLK_PWRCTL_HXT_EN;
    /* Set CLKO clock source and divider */
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_CLKOSEL_Msk)) |
                   (CLK_CLKO_SRC_EXT);
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk)) |
                   ((1) << CLK_CLKOCTL_FREQSEL_Pos) |
                   ((0) << CLK_CLKOCTL_DIV1EN_Pos) |
                   (CLK_CLKOCTL_CLKOEN_Msk);

    delay2s();
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN;

    SYS_LockReg();

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
