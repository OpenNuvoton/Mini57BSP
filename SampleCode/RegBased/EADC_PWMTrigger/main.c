/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for EADC PWM Trigger feature.
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

void EADC0_IRQHandler(void)
{
    if(EADC_GET_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk))
    {
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS_ADC0F_Msk); /* clear EADC0 interrupt flag */
        printf("Get EADC0 FIFO 0 data = %d\n", EADC_GET_CONV_DATA(EADC, EADC_EADC0_DAT0));
    }
}

uint32_t duty30, duty60;
void EPWM_IRQHandler(void)
{
    static int toggle = 0;  /* First two already fill into EPWM, so start from 30% */

    /* Update EPWM channel 0 duty */
    if(toggle == 0)
    {
        EPWM_SET_CMR(EPWM, 0, duty30);
    }
    else
    {
        EPWM_SET_CMR(EPWM, 0, duty60);
    }
    toggle ^= 1;
    /* Clear channel 0 period interrupt flag */
    EPWM->INTSTS = EPWM_INTSTS_PIF_Msk;
    printf("EPWM interrupt !!\n");
}

int main()
{
    char ch;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will configure PWM0 to trigger EADC0 channel 0 periodically
     * and print conversion results.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 EADC for PWM Trigger Sample Code |\n");
    printf("+-----------------------------------------+\n");

    /* Enable EADC clock */
    CLK->APBCLK |= CLK_APBCLK_ADCCKEN_Msk;

    /* Set EADC clock source to HIRC */
    CLK->CLKDIV  = (CLK->CLKDIV  & ~CLK_CLKDIV_ADCDIV_Msk)  | CLK_CLKDIV_EADC(4);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_ADCSEL_Msk) | CLK_EADC_SRC_HIRC;

    /* Reset IP */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    printf("Set GPB0 pin as EADC0 AIN0 input pin.\n");
    SYS->GPB_MFP = (SYS->GPB_MFP & ~(SYS_GPB_MFP_PB0MFP_Msk)) | SYS_GPB_MFP_PB0_ADC0_CH0;
    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE0_Msk)) | (GPIO_MODE_INPUT << GPIO_MODE_MODE0_Pos);
    PB->DINOFF |= GPIO_DINOFF_DINOFF0_Msk;

    /*--- Initial EPWM to trigger EADC */

    /* Enable EPWM clock */
    CLK->APBCLK |= CLK_APBCLK_EPWMCKEN_Msk;

    /* Set GPA0 multi-function pins for EPWM Channel0 */
    SYS->GPA_MFP = (SYS->GPA_MFP & ~(SYS_GPA_MFP_PA0MFP_Msk)) | SYS_GPA_MFP_PA0_EPWM_CH0;

    /* Set GPA0 as output mode */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);

    printf("Set EPWM channel 0 to trigger EADC.\n");
    /* EPWM channel 0 wave form of this sample changed between 30% and 60% duty ratio */
    EPWM->CLKDIV = 9;
    EPWM->CTL = (EPWM->CTL & ~EPWM_CTL_CNTTYPE_Msk) | (EPWM_CTL_CNTMODE_Msk);
    EPWM->CMPDAT[0] = 19660;
    EPWM->PERIOD = 65534;

    /* Save 30% duty setting */
    duty30 = EPWM->CMPDAT[0];
    /* Calculate 60% duty setting. */
    duty60 = duty30 * 2;

    /* Enable EPWM channel 0 period interrupt */
    EPWM->INTEN |= (EPWM_INTEN_PIEN_Msk);

    NVIC_EnableIRQ(EPWM_IRQn);

    /*--- Initial EADC */

    /* Enable EADC */
    /* Configure EADC0: channel 0 (AIN0), EPWM0 Rising trigger */
    /* Configure EADC conversion mode to Independent Simple Mode */
    EADC->CTL = (EADC->CTL & ~(EADC_CTL_ADC0CHSEL_Msk | EADC_CTL_ADC0HWTRGEN_Msk | EADC_CTL_ADCMODE_Msk)) |
                ((EADC_EADC0_0 << EADC_CTL_ADC0CHSEL_Pos) | (0x0 << EADC_CTL_ADCMODE_Pos) | EADC_CTL_ADC0HWTRGEN_Msk | EADC_CTL_ADCEN_Msk);
    EADC->TRGSOR = (EADC->TRGSOR & ~(EADC_TRGSOR_ADC0TRGSOR_Msk | EADC_TRGSOR_ADC0STADCSEL_Msk | EADC_TRGSOR_ADC0PWMTRGSEL_Msk)) |
                   (EADC_EPWM0_RISING_TRIGGER << EADC_TRGSOR_ADC0TRGSOR_Pos);


    /* Configure EADC sample time to 6 EADC clocks */
    EADC->SMPCNT = (EADC->SMPCNT & ~(EADC_SMPCNT_ADCSMPCNT_Msk)) |
                   (5 << EADC_SMPCNT_ADCSMPCNT_Pos);

    EADC_ENABLE_INT(EADC, EADC_CTL_ADC0IEN_Msk);

    NVIC_EnableIRQ(EADC0_IRQn);

    /* Begin to do EADC conversion. */
    ch = 0;

    /* Start EPWM module channel 0 */
    EPWM->CTL |= BIT0;

    /* Fill second duty setting immediately after EPWM start */
    EPWM_SET_CMR(EPWM, 0, duty60);

    while (ch != 'q')
    {
        printf("Press 'q' to quit.\n");
        ch = getchar();
    }

    NVIC_DisableIRQ(EPWM_IRQn);

    /* Disable EPWM clock */
    CLK->APBCLK &= ~CLK_APBCLK_EPWMCKEN_Msk;

    NVIC_DisableIRQ(EADC0_IRQn);

    /* Close EADC */
    EADC->CTL = (EADC->CTL & ~(EADC_CTL_ADCEN_Msk)) |
                ( (EADC_EADC0_7 << EADC_CTL_ADC0CHSEL_Pos) |                    /* Switching to channel Vss to save power */
                  ((EADC_EADC1_7-EADC_EADC1_0) << EADC_CTL_ADC1CHSEL_Pos) );    /* Switching to channel Vss to save power */
    SYS->IPRST1 |= SYS_IPRST1_ADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_ADCRST_Msk;

    /* Disable EADC clock */
    CLK->APBCLK &= ~CLK_APBCLK_ADCCKEN_Msk;

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
