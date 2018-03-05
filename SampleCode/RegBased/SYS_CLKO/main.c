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
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}


void delay2s()
{
    CLK_SysTickDelay(300000);  /* delay 300ms */
    CLK_SysTickDelay(300000);  /* delay 300ms */
    CLK_SysTickDelay(300000);  /* delay 300ms */
    CLK_SysTickDelay(300000);  /* delay 300ms */
    CLK_SysTickDelay(300000);  /* delay 300ms */
    CLK_SysTickDelay(300000);  /* delay 300ms */
    CLK_SysTickDelay(200000);  /* delay 200ms */
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* printf("\n\nPDID 0x%08X\n", SYS_ReadPDID()); */    /* Display PDID */
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

    /* CLK_EnableModuleClock(CLKO_MODULE); */
    CLK->APBCLK |= CLK_APBCLK_CLKOCKEN_Msk;

    printf("CLKO = HCLK / 1 = %dHz.\n", SystemCoreClock);

    /* CLK_EnableCKO(CLK_CLKO_SRC_HCLK, 0, 1); */
    /*      CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, NULL); */
    CLK->CLKSEL1 = (CLK->CLKSEL1 &~ CLK_CLKSEL1_CLKOSEL_Msk) | CLK_CLKO_SRC_HCLK;
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk))
                    | ((0) << CLK_CLKOCTL_FREQSEL_Pos)
                    | ((1) << CLK_CLKOCTL_DIV1EN_Pos);
    /*      Enable CKO clock source */
    CLK->CLKOCTL |= CLK_CLKOCTL_CLKOEN_Msk;

    delay2s();

    printf("CLKO = HCLK / 2^(0+1) = %dHz.\n", SystemCoreClock/2);

    /* CLK_EnableCKO(CLK_CLKO_SRC_HCLK, 0, 0); */
    /*      CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, NULL); */
    CLK->CLKSEL1 = (CLK->CLKSEL1 &~ CLK_CLKSEL1_CLKOSEL_Msk) | CLK_CLKO_SRC_HCLK;
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk))
                    | ((0) << CLK_CLKOCTL_FREQSEL_Pos)
                    | ((0) << CLK_CLKOCTL_DIV1EN_Pos);
    /*      Enable CKO clock source */
    CLK->CLKOCTL |= CLK_CLKOCTL_CLKOEN_Msk;

    delay2s();

    printf("CLKO = HCLK / 2^(2+1) = %dHz.\n", SystemCoreClock/8);

    /* CLK_EnableCKO(CLK_CLKO_SRC_HCLK, 2, 0); */
    /*      CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, NULL); */
    CLK->CLKSEL1 = (CLK->CLKSEL1 &~ CLK_CLKSEL1_CLKOSEL_Msk) | CLK_CLKO_SRC_HCLK;
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk))
                    | ((2) << CLK_CLKOCTL_FREQSEL_Pos)
                    | ((0) << CLK_CLKOCTL_DIV1EN_Pos);
    /*      Enable CKO clock source */
    CLK->CLKOCTL |= CLK_CLKOCTL_CLKOEN_Msk;

    delay2s();

    SYS_UnlockReg();

    printf("CLKO = HXT / 1 = %dHz.\n", __HXT);
    /* CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN); */
    CLK->PWRCTL |= CLK_PWRCTL_HXT_EN;
    
    /* CLK_EnableCKO(CLK_CLKO_SRC_EXT, 0, 1); */
    /*      CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, NULL); */
    CLK->CLKSEL1 = (CLK->CLKSEL1 &~ CLK_CLKSEL1_CLKOSEL_Msk) | CLK_CLKO_SRC_EXT;
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk))
                    | ((0) << CLK_CLKOCTL_FREQSEL_Pos)
                    | ((1) << CLK_CLKOCTL_DIV1EN_Pos);
    /*      Enable CKO clock source */
    CLK->CLKOCTL |= CLK_CLKOCTL_CLKOEN_Msk;

    delay2s();
    
    /* CLK_DisableXtalRC(CLK_PWRCTL_HXT_EN); */
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN;

    printf("CLKO = HXT / 2^(1+1) = %dHz.\n", __HXT/4);

    /* CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN); */
    CLK->PWRCTL |= CLK_PWRCTL_HXT_EN;

    /* CLK_EnableCKO(CLK_CLKO_SRC_EXT, 1, 0); */
    /*      CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, NULL); */
    CLK->CLKSEL1 = (CLK->CLKSEL1 &~ CLK_CLKSEL1_CLKOSEL_Msk) | CLK_CLKO_SRC_EXT;
    /*      CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = (CLK->CLKOCTL & ~(CLK_CLKOCTL_FREQSEL_Msk | CLK_CLKOCTL_DIV1EN_Msk))
                    | ((1) << CLK_CLKOCTL_FREQSEL_Pos)
                    | ((0) << CLK_CLKOCTL_DIV1EN_Pos);
    /*      Enable CKO clock source */
    CLK->CLKOCTL |= CLK_CLKOCTL_CLKOEN_Msk;

    delay2s();

    /* CLK_DisableXtalRC(CLK_PWRCTL_HXT_EN); */
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN;

    SYS_LockReg();

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
