/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for ECAP capture feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t u32Status;
uint32_t u32IC0Hold, u32IC1Hold, u32IC2Hold;

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

    /* Enable IP clock */
    /* CLK_EnableModuleClock(ECAP_MODULE); */
    CLK->APBCLK |= CLK_APBCLK_ECAPCKEN_Msk;

    /* SYS_ResetModule(ECAP_RST); */
    SYS->IPRST1 |= SYS_IPRST1_CAPRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_CAPRST_Msk;

    /* CLK_EnableModuleClock(TMR0_MODULE); */
    CLK->APBCLK |= CLK_APBCLK_TMR0CKEN_Msk;

    /* Select IP clock source */

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins GPB0 ~ GPB2 for ECAP input channel IC0 ~ IC2 */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~(SYS_GPB_MFP_PB0MFP_Msk | SYS_GPB_MFP_PB1MFP_Msk | SYS_GPB_MFP_PB2MFP_Msk))
                    | SYS_GPB_MFP_PB0_ECAP0 | SYS_GPB_MFP_PB1_ECAP1 | SYS_GPB_MFP_PB2_ECAP2;

    /* GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT); */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE0_Pos);

    /* GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT); */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE1_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE1_Pos);

    /* GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT); */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE2_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}


void ECAP_Init(void)
{
    /* Set ECAP clock source and divider */
    /* ECAP_SEL_TIMER_CLK_SRC(ECAP, ECAP_CAPTURE_TIMER_CLK_SRC_CAP_CLK); */
    ECAP->CTL1 = (ECAP->CTL1 & ~ECAP_CTL1_CNTSRC_Msk) | (ECAP_CAPTURE_TIMER_CLK_SRC_CAP_CLK << ECAP_CTL1_CNTSRC_Pos);

    /* ECAP_SEL_TIMER_CLK_DIV(ECAP, ECAP_CAPTURE_TIMER_CLKDIV_1); */
    ECAP->CTL1 = (ECAP->CTL1 & ~ECAP_CTL1_CAPDIV_Msk) | (ECAP_CAPTURE_TIMER_CLKDIV_1 << ECAP_CTL1_CAPDIV_Pos);

    /* Enable ECAP Input Channel 0 (IC0) */
    /* ECAP_ENABLE_INPUT_CHANNEL(ECAP, ECAP_CTL0_IC0EN_Msk); */
    ECAP->CTL0 |= ECAP_CTL0_IC0EN_Msk;

    /* Select ECAP IC0 source from ECAPx */
    /* ECAP_SEL_INPUT_SRC(ECAP, ECAP_IC0, ECAP_CAP_INPUT_SRC_ECAPX); */
    ECAP->CTL0 = ( ECAP->CTL0 & ~(ECAP_CTL0_CAP0SEL_Msk << (ECAP_IC0<<1)) ) |
                 ( (ECAP_CAP_INPUT_SRC_ECAPX << ECAP_CTL0_CAP0SEL_Pos) << (ECAP_IC0<<1) );

    /* Select IC0 detect rising edge */
    /* ECAP_SEL_CAPTURE_EDGE(ECAP, ECAP_IC0, ECAP_RISING_EDGE); */
    ECAP->CTL1 = ( ECAP->CTL1 & ~(ECAP_CTL1_CAPEDG0_Msk << (ECAP_IC0 << 1)) ) |
                 ( (ECAP_RISING_EDGE <<  ECAP_CTL1_CAPEDG0_Pos) << (ECAP_IC0 << 1) );

    /* Input Channel 0 interrupt enabled */
    /* ECAP_EnableINT(ECAP, ECAP_CTL0_CAPTF0IEN_Msk); */
    ECAP->CTL0 |= ECAP_CTL0_CAPTF0IEN_Msk;
    NVIC_EnableIRQ(ECAP_IRQn);

    /* Enable ECAP */
    /* ECAP_Open(ECAP, ECAP_RELOAD_FUNCTION); */
    ECAP->CTL0 = ECAP->CTL0 & ~(ECAP_CTL0_RLDEN_Msk | ECAP_CTL0_CMPEN_Msk);
    ECAP->CTL0 |= (ECAP_CTL0_CAPEN_Msk | (ECAP_RELOAD_FUNCTION << ECAP_CTL0_RLDEN_Pos));

    printf("Set ECAP: Input channel 0 from ECAP0 (PB0);\n");
    printf("          clock source HCLK; clock divider 1;\n");
    printf("          Rising edge; disable Compare; enable Reload.\n");
}


void ECAP_IRQHandler(void)
{
    /* Get input Capture status */
    /* u32Status = ECAP_GET_INT_STATUS(ECAP); */
    u32Status = ECAP->STS;

    /* Check input capture channel 0 flag */
    if((u32Status & ECAP_STS_CAPTF0_Msk) == ECAP_STS_CAPTF0_Msk)
    {
        /* Clear input capture channel 0 flag */
        /* ECAP_CLR_CAPTURE_FLAG(ECAP, ECAP_STS_CAPTF0_Msk); */
        ECAP->STS = ECAP_STS_CAPTF0_Msk;

        /* Get input capture counter hold value */
        /* u32IC0Hold = ECAP_GET_CNT_HOLD_VALUE(ECAP, ECAP_IC0); */
        u32IC0Hold = ECAP->HLD[ECAP_IC0] & 0xFFFFFF;
    }

    /* Check input capture channel 1 flag */
    if((u32Status & ECAP_STS_CAPTF1_Msk) == ECAP_STS_CAPTF1_Msk)
    {
        /* Clear input capture channel 1 flag */
        /* ECAP_CLR_CAPTURE_FLAG(ECAP, ECAP_STS_CAPTF1_Msk); */
        ECAP->STS = ECAP_STS_CAPTF1_Msk;

        /* Get input capture counter hold value */
        /* u32IC1Hold = ECAP_GET_CNT_HOLD_VALUE(ECAP, ECAP_IC1); */
        u32IC1Hold = ECAP->HLD[ECAP_IC1] & 0xFFFFFF;
    }

    /* Check input capture channel 2 flag */
    if((u32Status & ECAP_STS_CAPTF2_Msk) == ECAP_STS_CAPTF2_Msk)
    {
        /* Clear input capture channel 2 flag */
        /* ECAP_CLR_CAPTURE_FLAG(ECAP, ECAP_STS_CAPTF2_Msk); */
        ECAP->STS = ECAP_STS_CAPTF2_Msk;

        /* Get input capture counter hold value */
        /* u32IC2Hold = ECAP_GET_CNT_HOLD_VALUE(ECAP, ECAP_IC2); */
        u32IC2Hold = ECAP->HLD[ECAP_IC2] & 0xFFFFFF;
    }

    /* Check input capture compare-match flag */
    if((u32Status & ECAP_STS_CAPCMPF_Msk) == ECAP_STS_CAPCMPF_Msk)
    {
        /* Clear input capture compare-match flag */
        /* ECAP_CLR_CMP_MATCH_FLAG(ECAP); */
        ECAP->STS = ECAP_STS_CAPCMPF_Msk;
    }

    /* Check input capture overflow flag */
    if((u32Status & ECAP_STS_CAPOVF_Msk) == ECAP_STS_CAPOVF_Msk)
    {
        /* Clear input capture overflow flag */
        /* ECAP_CLR_OVF_FLAG(ECAP); */
        ECAP->STS = ECAP_STS_CAPOVF_Msk;
    }
}


void TMR0_IRQHandler(void)
{
    /* clear timer interrupt flag */
    /* TIMER_ClearIntFlag(TIMER0); */
    TIMER0->INTSTS = TIMER_INTSTS_TIF_Msk;

    /* GPIO_TOGGLE(PB4); */   /* generate signal as ECAP input */
    PB4 ^= 1;
}


int main()
{
    int i;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    /* printf("\n\nPDID 0x%08X\n", SYS_ReadPDID()); */    /* Display PDID */
    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will configure ECAP channel 0 to capture input square wave
     * and print capture results. The input square wave is generated by Timer0 and
     * GPIO output pin.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 ECAP Sample Code                 |\n");
    printf("+-----------------------------------------+\n");

    /*--- Initial ECAP */
    printf("Set PB4 pin as GPIO output pin to generate signal as ECAP input.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk)) | SYS_GPB_MFP_PB4_GPIO;

    /* GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT); */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE4_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE4_Pos);

    /* Clear ECAP interrupt status flags before start up-counting */
    /* ECAP->STS = ECAP_GET_INT_STATUS(ECAP); */
    ECAP->STS = ECAP->STS;

    ECAP_Init();

    /*--- Initial Timer to toggle ECAP input channel */
    printf("Set Timer0 to periodic 4Hz. It will toggle PB4 4 times per second.\n");
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 4);

    /* Enable timer interrupt */
    /* TIMER_EnableInt(TIMER0); */
    TIMER0->CTL |= TIMER_CTL_INTEN_Msk;

    NVIC_EnableIRQ(TMR0_IRQn);

    /*--- Initial global variables */
    u32Status = 0;
    u32IC0Hold = 0;
    u32IC1Hold = 0;
    u32IC2Hold = 0;
    printf("  >> Please connect PB0 and PB4 first << \n");
    printf("     Press any key to start ECAP test ...\n\n");
    getchar();

    /*--- Start both ECAP counter and Timer0 */
    printf("Start both ECAP counter and Timer0...\n");
    /* ECAP_CNT_START(ECAP); */
    ECAP->CTL0 |= ECAP_CTL0_CPTST_Msk;

    /* TIMER_Start(TIMER0); */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;

    i = 0;
    while(i < 10)
    {
        if(u32Status != 0)
        {
            /* Input Capture status is changed, and get a new hold value of input capture counter */
            printf("Input Captured !! Hold value = %8d, (CmpMatch: %s, Overflow: %s)\n", u32IC0Hold,
                (u32Status & ECAP_STS_CAPCMPF_Msk) ? "Yes" : "No",
                (u32Status & ECAP_STS_CAPOVF_Msk) ? "Yes" : "No" );
            u32Status = 0;
            i++;
        }
    }

    printf("\nSlow down the ECAP clock by set ECAP clock divider to 32.\n\n");
    /* ECAP_SEL_TIMER_CLK_DIV(ECAP, ECAP_CAPTURE_TIMER_CLKDIV_32); */
    ECAP->CTL1 = (ECAP->CTL1 & ~ECAP_CTL1_CAPDIV_Msk) | (ECAP_CAPTURE_TIMER_CLKDIV_32 << ECAP_CTL1_CAPDIV_Pos);

    /* ECAP_SET_CNT_VALUE(ECAP, 0); */    /* set ECAP counter to 0 for new ECAP clock */
    ECAP->CNT = 0 & 0xFFFFFF;

    i = 0;
    while(i < 10)
    {
        if(u32Status != 0)
        {
            /* Input Capture status is changed, and get a new hold value of input capture counter */
            printf("Input Captured !! Hold value = %8d, (CmpMatch: %s, Overflow: %s)\n", u32IC0Hold,
                (u32Status & ECAP_STS_CAPCMPF_Msk) ? "Yes" : "No",
                (u32Status & ECAP_STS_CAPOVF_Msk) ? "Yes" : "No" );
            u32Status = 0;
            i++;
        }
    }

    NVIC_DisableIRQ(TMR0_IRQn);

    /* TIMER_Close(TIMER0); */
    TIMER0->CTL = 0;
    TIMER0->EXTCTL = 0;

    /* CLK_DisableModuleClock(TMR0_MODULE); */
    CLK->APBCLK &= ~CLK_APBCLK_TMR0CKEN_Msk;

    /* ECAP_DisableINT(ECAP, ECAP_CTL0_CAPTF0IEN_Msk); */
    ECAP->CTL0 &= ~ECAP_CTL0_CAPTF0IEN_Msk;
    NVIC_DisableIRQ(ECAP_IRQn);

    /* ECAP_Close(ECAP); */
    ECAP->CTL0 &= ~ECAP_CTL0_CAPEN_Msk;

    /* CLK_DisableModuleClock(ECAP_MODULE); */
    CLK->APBCLK &= ~CLK_APBCLK_ECAPCKEN_Msk;

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/