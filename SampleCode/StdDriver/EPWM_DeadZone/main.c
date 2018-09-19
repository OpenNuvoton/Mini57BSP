/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:49p $
 * @brief    Demonstrate the EPWM dead-zone feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void EPWM_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
    if(++cnt == 100)
    {
        if(out)
            EPWM_EnableOutput(EPWM, 0x3F);
        else
            EPWM_DisableOutput(EPWM, 0x3F);
        out ^= 1;
        cnt = 0;
    }
    /* Clear channel 0 period interrupt flag */
    EPWM_ClearPeriodIntFlag(EPWM, 0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable 48MHz HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for 48MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* HCLK Clock source from HIRC */
    CLK_SetHCLK(CLK_HCLK_SRC_HIRC,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(EPWM_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Set GPA multi-function pins for EPWM Channel0~5 */
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk)) | SYS_GPA_MFP_PA0_EPWM_CH0;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA1MFP_Msk)) | SYS_GPA_MFP_PA1_EPWM_CH1;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA2MFP_Msk)) | SYS_GPA_MFP_PA2_EPWM_CH2;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA3MFP_Msk)) | SYS_GPA_MFP_PA3_EPWM_CH3;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA4MFP_Msk)) | SYS_GPA_MFP_PA4_EPWM_CH4;
    SYS->GPA_MFP = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA5MFP_Msk)) | SYS_GPA_MFP_PA5_EPWM_CH5;

    /* Set GPA0~5 as output mode */
    GPIO_SetMode(PA, 0x3F, GPIO_MODE_OUTPUT);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\nThis sample code will output EPWM channel to with different\n");
    printf("duty, enable dead zone function of all EPWM pairs.\n");
    printf("And also enable/disable EPWM output every 1 second.\n");
    /* EPWM0 frequency is 100Hz, duty 30% */
    EPWM_ConfigOutputChannel(EPWM, 0, 100, 30);
    EPWM_EnableDeadZone(EPWM, 0, 150);

    /* EPWM2 frequency is 100Hz, duty 50% */
    EPWM_ConfigOutputChannel(EPWM, 2, 100, 50);
    EPWM_EnableDeadZone(EPWM, 2, 200);

    /* EPWM4 frequency is 100Hz, duty 70% */
    EPWM_ConfigOutputChannel(EPWM, 4, 100, 70);
    EPWM_EnableDeadZone(EPWM, 4, 100);

    /* Enable complementary mode */
    EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM);

    /* Enable output of all EPWM channels */
    EPWM_EnableOutput(EPWM, 0x3F);

    /* Enable EPWM channel 0 period interrupt, use channel 0 to measure time. */
    EPWM_EnablePeriodInt(EPWM, 0, 0);
    NVIC_EnableIRQ(EPWM_IRQn);

    /* Start */
    EPWM_Start(EPWM, 0x3F);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
