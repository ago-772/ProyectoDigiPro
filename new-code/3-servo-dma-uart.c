/*
 * Conexiones:
 * - Potenciómetro 1 -> P1.30 (AD0.4)
 * - Potenciómetro 2 -> P1.31 (AD0.5)
 * - Potenciómetro 3 -> P0.25 (AD0.2)
 * - Servo 1 -> P0.9
 * - Servo 2 -> P0.8
 * - Servo 3 -> P0.7
 * - Botón (PULL-UP) -> P2.10 (EINT0)
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpdma.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

// Definiciones
#define SERVO_PIN   9   // P0.9   -> Servo 1
#define SERVO_PIN2  8   // P0.8   -> Servo 2
#define SERVO_PIN3  7   // P0.7   -> Servo 3

#define ADC_CH      4   // AD0.4 -> P1.30 -> Potenciómetro 1
#define ADC_CH5     5   // AD0.5 -> P1.31 -> Potenciómetro 2
#define ADC_CH2     2   // AD0.2 -> P0.25 -> Potenciómetro 3

// --- Definiciones para Grabación ---
#define PWM_PERIODO_US  20000   // 20ms
#define TIEMPO_GRAB_S   10      // 10 segundos
#define NUM_MUESTRAS    1000
// Variables globales
volatile uint16_t valorADC1 = 0;
volatile uint16_t valorADC2 = 0;
volatile uint16_t valorADC3 = 0;

volatile uint32_t pulso = 1500;
volatile uint32_t pulso2 = 1500;
volatile uint32_t pulso3 = 1500;

volatile uint32_t counter = 0;

// --- Variables para Grabación/Reproducción ---
volatile uint8_t systemMode = 0; // 0: Manual, 1: Grabando, 2: Reproduciendo

//volatile uint32_t recordedPulses[NUM_MUESTRAS][3];
volatile uint32_t recordedPulses[NUM_MUESTRAS][3] __attribute__ ((aligned(4)));
volatile uint32_t recordIndex = 0;

// --- Variables DMA ---
// volatile GPDMA_LLI_Type dmaLLI[NUM_MUESTRAS];
volatile GPDMA_LLI_Type dmaLLI[NUM_MUESTRAS] __attribute__ ((aligned(16)));

//++++++++++++++++++++++++ Config UART ++++++++++++++++++++++++
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

//++++++++++++++++++++++++ Config UART ++++++++++++++++++++++++
void configUART(void) {
    UART_CFG_Type uart_cfg;
    UART_FIFO_CFG_Type fifo_cfg;

    UART_ConfigStructInit(&uart_cfg);
    UART_Init((LPC_UART_TypeDef *) LPC_UART0, &uart_cfg);

    UART_FIFOConfigStructInit(&fifo_cfg);
    UART_FIFOConfig((LPC_UART_TypeDef *) LPC_UART0, &fifo_cfg);

    UART_TxCmd((LPC_UART_TypeDef *) LPC_UART0, ENABLE);
}

//++++++++++++++++++++++++Configuración GPIO++++++++++++++++++++++++
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0; // GPIO
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    // Servo 1
    PinCfg.Pinnum = SERVO_PIN;
    PINSEL_ConfigPin(&PinCfg);

    // Servo 2
    PinCfg.Pinnum = SERVO_PIN2;
    PINSEL_ConfigPin(&PinCfg);

    // Servo 3
    PinCfg.Pinnum = SERVO_PIN3;
    PINSEL_ConfigPin(&PinCfg);

    // Configurar como salida
    GPIO_SetDir(0, (1<<SERVO_PIN) | (1<<SERVO_PIN2) | (1<<SERVO_PIN3), 1);
}

//++++++++++++++++++++++++Configuración ADC++++++++++++++++++++++++
void ConfADC(void)
{
    PINSEL_CFG_Type PinCfg;

    // Potenciómetro 1 -> AD0.4 (P1.30)
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // Potenciómetro 2 -> AD0.5 (P1.31)
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);

    // Potenciómetro 3 -> AD0.2 (P0.25)
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 25;
    PinCfg.Funcnum = 1;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 1000);
    ADC_BurstCmd(LPC_ADC, ENABLE);

    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH5, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH2, ENABLE);

    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH5, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH2, ENABLE);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
    // NVIC_EnableIRQ(ADC_IRQn); // Se habilita/deshabilita desde EINT
}

//++++++++++++++++++++++++Interrupción ADC++++++++++++++++++++++++
void ADC_IRQHandler(void)
{
    // Solo actualiza 'pulso' si el DMA no está activo (Modo Manual o Grabando)
    if (systemMode != 2)
    {
        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
        {
            valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);

            if (valorADC1 >= 1948 && valorADC1 < 2147)
                pulso = 1500;
            else if (valorADC1 < 1948)
                pulso = 1000 + (uint32_t)(((float)valorADC1 / 1947.0f) * 500.0f);
            else
                pulso = 1500 + (uint32_t)(((float)(valorADC1 - 2147) / 1948.0f) * 500.0f);

            TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
        }

        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH5, 1))
        {
            valorADC2 = ADC_ChannelGetData(LPC_ADC, ADC_CH5);

            if (valorADC2 >= 1948 && valorADC2 < 2147)
                pulso2 = 1500;
            else if (valorADC2 < 1948)
                pulso2 = 1000 + (uint32_t)(((float)valorADC2 / 1947.0f) * 500.0f);
            else
                pulso2 = 1500 + (uint32_t)(((float)(valorADC2 - 2147) / 1948.0f) * 500.0f);

            TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);
        }

        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH2, 1))
        {
            valorADC3 = ADC_ChannelGetData(LPC_ADC, ADC_CH2);

            if (valorADC3 >= 1948 && valorADC3 < 2147)
                pulso3 = 1500;
            else if (valorADC3 < 1948)
                pulso3 = 1000 + (uint32_t)(((float)valorADC3 / 1947.0f) * 500.0f);
            else
                pulso3 = 1500 + (uint32_t)(((float)(valorADC3 - 2147) / 1948.0f) * 500.0f);

            TIM_UpdateMatchValue(LPC_TIM2, 3, pulso3);
        }
    } else {
        // Si estamos en modo DMA, solo limpia los flags para evitar que se bloquee la IRQ
        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1)) ADC_ChannelGetData(LPC_ADC, ADC_CH);
        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH5, 1)) ADC_ChannelGetData(LPC_ADC, ADC_CH5);
        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH2, 1)) ADC_ChannelGetData(LPC_ADC, ADC_CH2);
    }
}

//++++++++++++++++++++++++ Timer2 PWM software ++++++++++++++++++++++++
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000; // Periodo de 20ms
    TIM_ConfigMatch(LPC_TIM2, &match0);

    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    TIM_MATCHCFG_Type match2;
    match2.MatchChannel = 2;
    match2.IntOnMatch = ENABLE;
    match2.ResetOnMatch = DISABLE;
    match2.StopOnMatch = DISABLE;
    match2.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match2.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match2);

    TIM_MATCHCFG_Type match3;
    match3.MatchChannel = 3;
    match3.IntOnMatch = ENABLE;
    match3.ResetOnMatch = DISABLE;
    match3.StopOnMatch = DISABLE;
    match3.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match3.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match3);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// --- Configuración Timer0 para Grabación ---
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1; // 1us
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE; // Reinicia el timer cada 20ms
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = PWM_PERIODO_US; // 20000us = 20ms
    TIM_ConfigMatch(LPC_TIM0, &match0);

    NVIC_EnableIRQ(TIMER0_IRQn);
    // NO se inicia aquí, se inicia al entrar en modo grabación
    // TIM_Cmd(LPC_TIM0, ENABLE);
}

// --- Configuración EINT0 para cambiar de modo ---
void ConfEINT(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Funcnum = 1; // EINT0
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP; // Asume botón a GND
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    EXTI_Init();
    EXTI_SetMode(EXTI_EINT0, EXTI_MODE_EDGE_SENSITIVE);
    NVIC_EnableIRQ(EINT0_IRQn);
}

// --- Configuración DMA para Reproducción ---
void ConfDMA_Playback(void)
{
/*
recordedPulses[i][0] → servo1
recordedPulses[i][1] → servo2
recordedPulses[i][2] → servo3
*/
    GPDMA_Channel_CFG_Type dmaCfg;

    GPDMA_Init();

    // 1. Construir la Lista Enlazada (LLI)
    for (int i = 0; i < NUM_MUESTRAS; i++)
    {
        dmaLLI[i].SrcAddr = (uint32_t)&recordedPulses[i][0];

        // Destino: Primer registro Match del Timer2 (MR1)
        // El DMA escribirá en MR1, MR2, MR3 secuencialmente gracias a 'Dest Increment'
        dmaLLI[i].DstAddr = (uint32_t)&(LPC_TIM2->MR1);

        // Apunta al siguiente item LLI
        dmaLLI[i].NextLLI = (uint32_t)&dmaLLI[i + 1];

        // Configuración de control del DMA para esta transferencia
        dmaLLI[i].Control =
            (3 << 0)     |   // TransferSize = 3 words (MR1, MR2, MR3)
            (2 << 18)    |   // Source width = 32 bits
            (2 << 21)    |   // Dest   width = 32 bits
            (0 << 12)    |   // Source burst = 1
            (0 << 15)    |   // Dest   burst = 1
            (1 << 26)    |   // SI = Source increment
            (1 << 27);       // DI = Dest increment

    }

    // 2. Hacer que la LLI sea circular (la última apunta a la primera)
    dmaLLI[NUM_MUESTRAS - 1].NextLLI = (uint32_t)&dmaLLI[0];

    // 3. Configurar el Canal 0 del DMA
    dmaCfg.ChannelNum = 0;
    dmaCfg.SrcMemAddr = 0; // No se usa, definido por LLI
    dmaCfg.DstMemAddr = 0; // No se usa, definido por LLI
    dmaCfg.TransferSize = 0; // No se usa, definido por LLI
    dmaCfg.TransferWidth = 0; // No se usa, definido por LLI
    dmaCfg.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memoria a Periférico
    dmaCfg.SrcConn = 0;
    dmaCfg.DstConn = GPDMA_CONN_MAT0_0; // Disparo por Timer2 Match 0
    dmaCfg.DMALLI = (uint32_t)&dmaLLI[0]; // Apunta al inicio de la LLI

    GPDMA_Setup(&dmaCfg);

    // NO se habilita aquí, se habilita al entrar en modo reproducción
    // GPDMA_ChannelCmd(0, ENABLE);
}


// Interrupción Timer2
void TIMER2_IRQHandler(void)
//genera el PWM
//Cada 1 segundo: manda mensaje por UART. - pulsos

{
    // MATCH0 -> reinicia ciclo PWM
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        counter++;

        // Envía estado por UART cada ~1 segundo (50 * 20ms)
        if (counter >= 50)
        {
            char mensaje[128];

            if (systemMode == 0) {
                sprintf(mensaje, "Modo: Manual (ADC)\r\n");
            } else if (systemMode == 1) {
                sprintf(mensaje, "Modo: Grabando... Muestra: %u/%u\r\n", recordIndex, NUM_MUESTRAS);
            } else {
                sprintf(mensaje, "Modo: Reproduciendo (DMA)...\r\n");
            }

            UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)mensaje, strlen(mensaje), BLOCKING);
            counter = 0;
        }

        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);

        // Pone todos los pines de servo en ALTO al inicio del ciclo
        GPIO_SetValue(0, (1<<SERVO_PIN));
        GPIO_SetValue(0, (1<<SERVO_PIN2));
        GPIO_SetValue(0, (1<<SERVO_PIN3));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN)); // Pone en BAJO al final del pulso
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN2));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR3_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR3_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN3));
    }
}

//++++++++++++++++++++++++ Interrupción Timer0 (Grabación) ++++++++++++++++++++++++
//Cada 20 ms guarda un trío de pulsos [pulso, pulso2, pulso3] en recordedPulses.
// 1000 muestras
void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

        if (systemMode == 1) // Modo Grabación
        {
            if (recordIndex < NUM_MUESTRAS)
            {
                // Almacena los pulsos actuales (leídos por el ADC)
                recordedPulses[recordIndex][0] = pulso;
                recordedPulses[recordIndex][1] = pulso2;
                recordedPulses[recordIndex][2] = pulso3;
                recordIndex++;

                if (recordIndex >= NUM_MUESTRAS){
                    recordIndex = 0;
                }
            }
        }
    }
}

//++++++++++++++++++++++++ Interrupción EINT0 (Cambio de Modo) ++++++++++++++++++++++++
void EINT0_IRQHandler(void)
{
    // Simple retardo anti-rebote
    for (volatile int i = 0; i < 1000000; i++);

    EXTI_ClearEXTIFlag(EXTI_EINT0);

    systemMode = (systemMode + 1) % 3; // Cicla 0 -> 1 -> 2 -> 0

    if (systemMode == 0) // Manual
    {
        GPDMA_ChannelCmd(0, DISABLE); // Detiene DMA
        TIM_Cmd(LPC_TIM0, DISABLE);   // Detiene Timer0 (grabación)
        NVIC_EnableIRQ(ADC_IRQn);     // Habilita control por ADC
    }
    else if (systemMode == 1) // Grabando
    {
        GPDMA_ChannelCmd(0, DISABLE); // Detiene DMA
        recordIndex = 0;              // Reinicia índice de grabación
        NVIC_EnableIRQ(ADC_IRQn);     // Mantiene control por ADC
        TIM_Cmd(LPC_TIM0, ENABLE);    // Inicia Timer0 (tick de grabación)
    }
    else // systemMode == 2: Reproduciendo
    {
        TIM_Cmd(LPC_TIM0, DISABLE);   // Detiene Timer0 (grabación)
        NVIC_DisableIRQ(ADC_IRQn);    // Deshabilita control por ADC
        GPDMA_ChannelCmd(0, ENABLE);  // Inicia DMA
    }
}


// Main
int main(void)
{
    ConfGPIO();
    ConfADC();
    ConfTIMER2();
    ConfTIMER0();
    ConfEINT();
    ConfDMA_Playback(); // Configura el DMA y la LLI
    configGPIO_UART();
    configUART();

    // Inicia en Modo Manual (ADC habilitado)
    systemMode = 0;
    NVIC_EnableIRQ(ADC_IRQn);

    while (1)
    {

    }
}
