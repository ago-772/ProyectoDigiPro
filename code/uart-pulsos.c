/*
 * Probando facilmente 2 potenciómetros con leds y UART
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_uart.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define SERVO_PIN   9     // P0.9
#define SERVO_PIN2  8     // P0.8
#define ADC_CH      4     // AD0.4 (P1.30)
#define ADC_CH5     5     // AD0.5 (P1.31)

#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

volatile uint16_t valorADC1 = 0;
volatile uint16_t valorADC2 = 0;

volatile uint32_t pulso = 0;
volatile uint32_t pulso2 = 0;

volatile uint32_t counter = 0;

// ---------------- UART ----------------
void configGPIO_UART(void) {
    PINSEL_CFG_Type psel_conf;

    psel_conf.Portnum = 0;
    psel_conf.Pinmode = PINSEL_PINMODE_TRISTATE;
    psel_conf.OpenDrain = PINSEL_PINMODE_NORMAL;

    // TXD0 -> P0.2
    psel_conf.Pinnum = 2;
    psel_conf.Funcnum = 1;
    PINSEL_ConfigPin(&psel_conf);

    // RXD0 -> P0.3
    psel_conf.Pinnum = 3;
    psel_conf.Funcnum = 1;
    PINSEL_ConfigPin(&psel_conf);
}

void configUART(void) {
    UART_CFG_Type uart_cfg;
    UART_FIFO_CFG_Type fifo_cfg;

    UART_ConfigStructInit(&uart_cfg);
    UART_Init((LPC_UART_TypeDef *)LPC_UART0, &uart_cfg);

    UART_FIFOConfigStructInit(&fifo_cfg);
    UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &fifo_cfg);

    UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);
}

// ---------------- GPIO ----------------
void ConfGPIO(void) {
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = 0;

    PINSEL_ConfigPin(&PinCfg);

    GPIO_SetDir(0, (1 << SERVO_PIN), 1);
    GPIO_SetDir(0, (1 << SERVO_PIN2), 1);
    GPIO_SetDir(3, (1 << 25), 1);
    GPIO_SetDir(3, (1 << 26), 1);
}

// ---------------- ADC ----------------
void ConfADC(void) {
    PINSEL_CFG_Type PinCfg;

    // P1.30 -> AD0.4
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // P1.31 -> AD0.5
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 1000);
    ADC_BurstCmd(LPC_ADC, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH5, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH5, ENABLE);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
    NVIC_EnableIRQ(ADC_IRQn);
}

// ---------------- ADC IRQ ----------------
void ADC_IRQHandler(void) {
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1)) {
        valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);
        if (valorADC1 >= 1948 && valorADC1 < 2147)
            pulso = 1500;
        else if (valorADC1 < 1948)
            pulso = 1000 + ((uint32_t)valorADC1 * 500 / 1947);
        else
            pulso = 1500 + ((uint32_t)valorADC1 * 500 / 1947);

        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }

    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH5, 1)) {
        valorADC2 = ADC_ChannelGetData(LPC_ADC, ADC_CH5);
        if (valorADC2 >= 1900 && valorADC2 < 2250)
            pulso2 = 1500;
        else if (valorADC2 < 1900)
            pulso2 = 1000 + ((uint32_t)valorADC2 * 500 / 1947);
        else
            pulso2 = 1500 + ((uint32_t)valorADC2 * 500 / 1947);

        TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);

    }

}

// ---------------- TIMER2 ----------------
void ConfTIMER2(void) {
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;

    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type match0 = {0};
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    TIM_MATCHCFG_Type match1 = {0};
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    TIM_MATCHCFG_Type match2 = {0};
    match2.MatchChannel = 2;
    match2.IntOnMatch = ENABLE;
    match2.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match2);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// ---------------- TIMER2 IRQ ----------------
void TIMER2_IRQHandler(void) {
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT)) {
    	counter++;
    	if(counter >= 50){

    	    char mensaje[64];
    	    sprintf(mensaje, "Pulsos: S1=%lu us  S2=%lu us\r\n", pulso, pulso2);
    	    UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)mensaje, strlen(mensaje), NONE_BLOCKING);

    	    counter = 0;
    	}

        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1 << SERVO_PIN));
        GPIO_SetValue(0, (1 << SERVO_PIN2));
    }
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1 << SERVO_PIN));
    }
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
        GPIO_ClearValue(0, (1 << SERVO_PIN2));
    }
}

// ---------------- MAIN ----------------
int main(void) {
    ConfGPIO();
    ConfADC();
    ConfTIMER2();
    configGPIO_UART();
    configUART();

    while (1) {

        }
    }