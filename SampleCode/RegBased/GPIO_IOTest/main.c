/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/19 6:46p $
 * @brief    Sample code for GPIO I/O feature.
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


/**
 * @brief       PortA/PortB/PortC/PortD IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortA/PortB/PortC/PortD default IRQ, declared in startup_Mini57Series.s.
 */
uint32_t PAIntFlag=0, PBIntFlag=0, PCIntFlag=0, PDIntFlag=0;
void GPABCD_IRQHandler(void)
{
    /* PAIntFlag = GPIO_GET_INT_FLAG(PA, 0xF); */
    PAIntFlag = PA->INTSRC & 0xF;

    /* PBIntFlag = GPIO_GET_INT_FLAG(PB, 0xF); */
    PBIntFlag = PB->INTSRC & 0xF;

    /* PCIntFlag = GPIO_GET_INT_FLAG(PC, 0xF); */
    PCIntFlag = PC->INTSRC & 0xF;

    /* PDIntFlag = GPIO_GET_INT_FLAG(PD, 0xF); */
    PDIntFlag = PD->INTSRC & 0xF;

    /* clear GPIO interrupt flag */
    /* GPIO_CLR_INT_FLAG(PA, PAIntFlag); */
    PA->INTSRC = PAIntFlag;

    /* GPIO_CLR_INT_FLAG(PB, PBIntFlag); */
    PB->INTSRC = PBIntFlag;

    /* GPIO_CLR_INT_FLAG(PC, PCIntFlag); */
    PC->INTSRC = PCIntFlag;

    /* GPIO_CLR_INT_FLAG(PD, PDIntFlag); */
    PD->INTSRC = PDIntFlag;
}


/**
  * @brief      This function execute delay function.
  * @param[in]  us      Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                     50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @return     None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  */
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


int main()
{
    int32_t i32Err;

    SYS_Init();

    /* Init USCI UART0 to 115200-8n1 for print message */
    UUART0_Init();

    printf("\n\nPDID 0x%08X\n", (unsigned int)(SYS->PDID & SYS_PDID_PDID_Msk)); /* Display PDID */

    printf("CPU @ %dHz\n", SystemCoreClock);        /* Display System Core Clock */

    /*
     * This sample code will use GPIO driver to control the GPIO pin direction and
     * the high/low state, and show how to use GPIO interrupts.
     */
    printf("+-----------------------------------------+\n");
    printf("| Mini57 GPIO Driver Sample Code          |\n");
    printf("+-----------------------------------------+\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PB.0 and PC.2 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Config multiple function to GPIO mode for PB0 and PC2 */
    SYS->GPB_MFP = (SYS->GPB_MFP & ~SYS_GPB_MFP_PB0MFP_Msk) | SYS_GPB_MFP_PB0_GPIO;
    SYS->GPC_MFP = (SYS->GPC_MFP & ~SYS_GPC_MFP_PC2MFP_Msk) | SYS_GPC_MFP_PC2_GPIO;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
    PC->MODE = (PC->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE2_Pos);

    /* Enable PC2 interrupt by rising edge trigger */
    PC->INTEN = (PC->INTEN & ~(GPIO_INT_BOTH_EDGE << 2)) | (GPIO_INT_RISING << 2);
    PC->INTTYPE = (PC->INTTYPE & ~(BIT0 << 2)) | (GPIO_INTTYPE_EDGE << 2);

    NVIC_EnableIRQ(GP_IRQn);                    /* Enable GPIO NVIC */

    PAIntFlag = 0;
    PBIntFlag = 0;
    PCIntFlag = 0;
    PDIntFlag = 0;
    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    PB0 = 0;                /* Output low */
    _CLK_SysTickDelay(10);  /* wait for IO stable */
    if (PC2 != 0)           /* check if the PB3 state is low */
    {
        i32Err = 1;
    }

    PB0 = 1;                /* Output high */
    _CLK_SysTickDelay(10);  /* wait for IO stable */
    if (PC2 != 1)           /* check if the PB3 state is high */
    {
        i32Err = 1;
    }

    /* show the result */
    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure PB.0 and PC.2 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    printf("  Check Interrupt Flag PA=0x%08X, PB=0x%08X, PC=0x%08X, PD=0x%08X\n",
           PAIntFlag, PBIntFlag, PCIntFlag, PDIntFlag);

    /* Configure PB.0 to default Input mode */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE0_Pos);

    /* Configure PC.2 to default Input mode */
    PC->MODE = (PC->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_INPUT << GPIO_MODE_MODE2_Pos);

    printf("=== THE END ===\n\n");
    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
