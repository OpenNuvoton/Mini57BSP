/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for ECAP compare-match feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t u32Status;
volatile uint32_t u32IC0Hold, u32IC1Hold, u32IC2Hold;

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

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_ECAPCKEN_Msk;

    SYS->IPRST1 |= SYS_IPRST1_CAPRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_CAPRST_Msk;

    CLK->APBCLK |= CLK_APBCLK_TMR0CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins GPB0 ~ GPB2 for ECAP input channel IC0 ~ IC2 */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~(SYS_GPB_MFP_PB0MFP_Msk | SYS_GPB_MFP_PB1MFP_Msk | SYS_GPB_MFP_PB2MFP_Msk)) |
                   (SYS_GPB_MFP_PB0_ECAP_P0 | SYS_GPB_MFP_PB1_ECAP_P1 | SYS_GPB_MFP_PB2_ECAP_P2);

    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk)) |
               (GPIO_MODE_INPUT << GPIO_MODE_MODE0_Pos) |
               (GPIO_MODE_INPUT << GPIO_MODE_MODE1_Pos) |
               (GPIO_MODE_INPUT << GPIO_MODE_MODE2_Pos);

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


void ECAP_Init(void)
{
    /* Set ECAP clock source and divider */
    /* Select IC0 detect rising edge */
    ECAP->CTL1 = (ECAP->CTL1 & ~(ECAP_CTL1_CNTSRC_Msk | ECAP_CTL1_CAPDIV_Msk | ECAP_CTL1_CAPEDG0_Msk)) |
                 (ECAP_CAPTURE_TIMER_CLK_SRC_CAP_CLK << ECAP_CTL1_CNTSRC_Pos) |
                 (ECAP_CAPTURE_TIMER_CLKDIV_64 << ECAP_CTL1_CAPDIV_Pos) |
                 (ECAP_RISING_EDGE <<  ECAP_CTL1_CAPEDG0_Pos);

    /* Enable ECAP Input Channel 0 (IC0) */
    /* Select ECAP IC0 source from ECAPx */
    /* Input Channel 0 interrupt enabled */
    /* Set Compare-Match feature */
    /* Enable ECAP */
    ECAP->CTL0 = (ECAP->CTL0 & ~(ECAP_CTL0_CAP0SEL_Msk | ECAP_CTL0_CMPCLR_Msk | ECAP_CTL0_CPTCLR_Msk)) |
                 (ECAP_CAP_INPUT_SRC_ECAPX << ECAP_CTL0_CAP0SEL_Pos) |
                 (ECAP_CTL0_IC0EN_Msk) |
                 (ECAP_CTL0_CAPTF0IEN_Msk) |
                 (ECAP_CNT_CLR_BY_CAMCMPF << ECAP_CTL0_CMPCLR_Pos) |
                 (ECAP_CTL0_CAPCMPIEN_Msk);

    NVIC_EnableIRQ(ECAP_IRQn);

    ECAP_SET_CNT_CMP(ECAP, 500000);

    /* Enable ECAP */
    ECAP->CTL0 = (ECAP->CTL0 & ~(ECAP_CTL0_RLDEN_Msk | ECAP_CTL0_CMPEN_Msk)) |
                 (ECAP_CTL0_CAPEN_Msk | (ECAP_COMPARE_FUNCTION << ECAP_CTL0_RLDEN_Pos));

    printf("Set ECAP: Input channel 0 from ECAP0 (PB0);\n");
    printf("          clock source HCLK; clock divider 64;\n");
    printf("          Rising edge; enable Compare; disable Reload.\n");
    printf("          compare-match count 500000.\n");
}


void ECAP_IRQHandler(void)
{
    /* Get input Capture status */
    u32Status = ECAP_GET_INT_STATUS(ECAP);

    /* Check input capture channel 0 flag */
    if((u32Status & ECAP_STS_CAPTF0_Msk) == ECAP_STS_CAPTF0_Msk)
    {
        /* Clear input capture channel 0 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP, ECAP_STS_CAPTF0_Msk);

        /* Get input capture counter hold value */
        u32IC0Hold = ECAP_GET_CNT_HOLD_VALUE(ECAP, ECAP_IC0);
    }

    /* Check input capture channel 1 flag */
    if((u32Status & ECAP_STS_CAPTF1_Msk) == ECAP_STS_CAPTF1_Msk)
    {
        /* Clear input capture channel 1 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP, ECAP_STS_CAPTF1_Msk);

        /* Get input capture counter hold value */
        u32IC1Hold = ECAP_GET_CNT_HOLD_VALUE(ECAP, ECAP_IC1);
    }

    /* Check input capture channel 2 flag */
    if((u32Status & ECAP_STS_CAPTF2_Msk) == ECAP_STS_CAPTF2_Msk)
    {
        /* Clear input capture channel 2 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP, ECAP_STS_CAPTF2_Msk);

        /* Get input capture counter hold value */
        u32IC2Hold = ECAP_GET_CNT_HOLD_VALUE(ECAP, ECAP_IC2);
    }

    /* Check input capture compare-match flag */
    if((u32Status & ECAP_STS_CAPCMPF_Msk) == ECAP_STS_CAPCMPF_Msk)
    {
        /* Clear input capture compare-match flag */
        ECAP_CLR_CMP_MATCH_FLAG(ECAP);
    }

    /* Check input capture overflow flag */
    if((u32Status & ECAP_STS_CAPOVF_Msk) == ECAP_STS_CAPOVF_Msk)
    {
        /* Clear input capture overflow flag */
        ECAP_CLR_OVF_FLAG(ECAP);
    }
}


void TMR0_IRQHandler(void)
{
    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER0);

    GPIO_TOGGLE(PB4);   /* generate signal as ECAP input */
}


int main()
{
    int i;
    uint32_t u32StatusCmpf, u32StatusOvf;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will demonstrate ECAP capture and compare match function
     * by monitoring the capture result of ECAP channel 0.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 ECAP Compare Match Sample Code   |\n");
    printf("+-----------------------------------------+\n");

    /*--- Initial ECAP */
    printf("Set PB4 pin as GPIO output pin to generate signal as ECAP input.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB4MFP_Msk)) | SYS_GPB_MFP_PB4_GPIO;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE4_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE4_Pos);

    /* Clear ECAP interrupt status flags before start up-counting */
    ECAP->STS = ECAP_GET_INT_STATUS(ECAP);

    ECAP_Init();

    /*--- Initial Timer to toggle ECAP input channel */
    printf("Set Timer0 to periodic 4Hz. It will toggle PB4 4 times per second.\n");
    TIMER0->CTL = TIMER_PERIODIC_MODE | (4-1);
    TIMER0->CMP = __HXT / 4;

    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);

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
    ECAP_CNT_START(ECAP);

    TIMER_Start(TIMER0);

    i = 0;
    while(i < 10)
    {
        if(u32Status != 0)
        {
            if (u32Status & ECAP_STS_CAPTF0_Msk)
            {
                /* Input Capture status is changed, and get a new hold value of input capture counter */
                u32StatusCmpf = u32Status & ECAP_STS_CAPCMPF_Msk;
                u32StatusOvf  = u32Status & ECAP_STS_CAPOVF_Msk;
                printf("Input Captured !! Hold value = %8d, (CmpMatch: %s, Overflow: %s)\n", u32IC0Hold,
                    u32StatusCmpf ? "Yes" : "No",
                    u32StatusOvf ? "Yes" : "No" );
            }
            else if (u32Status & ECAP_STS_CAPCMPF_Msk)
                printf("Compare-Match !! Compare-Match count = %d\n", ECAP_GET_CNT_CMP(ECAP));
            u32Status = 0;
            i++;
        }
    }

    printf("\nSet ECAP Capture Compare Value to 300000.\n\n");

    ECAP_SET_CNT_CMP(ECAP, 300000);

    ECAP_SET_CNT_VALUE(ECAP, 0);    /* set ECAP counter to 0 for new ECAP clock */

    i = 0;
    while(i < 10)
    {
        if(u32Status != 0)
        {
            if (u32Status & ECAP_STS_CAPTF0_Msk)
            {
                /* Input Capture status is changed, and get a new hold value of input capture counter */
                u32StatusCmpf = u32Status & ECAP_STS_CAPCMPF_Msk;
                u32StatusOvf  = u32Status & ECAP_STS_CAPOVF_Msk;
                printf("Input Captured !! Hold value = %8d, (CmpMatch: %s, Overflow: %s)\n", u32IC0Hold,
                    u32StatusCmpf ? "Yes" : "No",
                    u32StatusOvf ? "Yes" : "No" );
            }
            else if (u32Status & ECAP_STS_CAPCMPF_Msk)
                printf("Compare-Match !! Compare-Match count = %d. Reset capture counter to 0.\n", ECAP_GET_CNT_CMP(ECAP));
            u32Status = 0;
            i++;
        }
    }

    NVIC_DisableIRQ(TMR0_IRQn);

    /* Close TIMER0 */
    TIMER0->CTL = 0;
    TIMER0->EXTCTL = 0;

    CLK->APBCLK &= ~CLK_APBCLK_TMR0CKEN_Msk;

    ECAP->CTL0 &= ~ECAP_CTL0_CAPTF0IEN_Msk;
    NVIC_DisableIRQ(ECAP_IRQn);

    ECAP->CTL0 &= ~ECAP_CTL0_CAPEN_Msk;

    CLK->APBCLK &= ~CLK_APBCLK_ECAPCKEN_Msk;

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
