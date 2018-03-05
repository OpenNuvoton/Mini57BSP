/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 17/05/02 7:13p $
 * @brief    Demonstrate the BPWM dead-zone feature.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"


void BPWM0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
    if(++cnt == 100) {
        if(out)
        {
            BPWM_EnableOutput(BPWM, 0x3);
            GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);
            GPIO_SetMode(PB, BIT2, GPIO_MODE_OUTPUT);
        }
        else
        {
            BPWM_DisableOutput(BPWM, 0x3);
            GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
            GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
        }
        out ^= 1;
        cnt = 0;
    }
    /* Clear channel 0 period interrupt flag */
    BPWM_ClearPeriodIntFlag(BPWM, 0);
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
    CLK_EnableModuleClock(BPWM_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);

    /* Set GPD5 as output mode and GPD6 as Input mode */
    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT6, GPIO_MODE_INPUT);

    /* Set GPC0 multi-function pins for BPWM Channel0 */
    SYS->GPC_MFP = (SYS->GPC_MFP & (~SYS_GPC_MFP_PC0MFP_Msk));
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_BPWM_CH0;
    /* Set GPB2 multi-function pins for BPWM Channel1 */
    SYS->GPB_MFP = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB2MFP_Msk));
    SYS->GPB_MFP |= SYS_GPB_MFP_PB2_BPWM_CH1;

    /* Set GPC0, GPB2 as output mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT2, GPIO_MODE_OUTPUT);

    /* Lock protected registers */
    SYS_LockReg();
}


int main()
{
    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART_Open(UUART0, 115200);

    printf("\nThis sample code will output BPWM channel 0 to with different\n");
    printf("frequency and duty, enable dead zone function of all BPWM pairs.\n");
    printf("And also enable/disable BPWM output every 1 second.\n");
    /* BPWM0 frequency is 100Hz, duty 30% */
    BPWM_ConfigOutputChannel(BPWM, 0, 100, 30);
    BPWM_EnableDeadZone(BPWM, 0, 100);

    /* Enable output of all BPWM channels */
    BPWM_EnableOutput(BPWM, 0x3);

    /* Enable BPWM channel 0 period interrupt, use channel 0 to measure time. */
    BPWM_EnablePeriodInt(BPWM, 0, 0);
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM_Start(BPWM, 0x3);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
