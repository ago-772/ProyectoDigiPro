/*
 * Proyecto: Control de 1 servomotor con 1 potenciómetro - LPC1769
 *
 * Descripción:
 * Control manual, grabación y reproducción de un solo servomotor.
 * - Modo 0 (Manual): Controlado por el Potenciómetro 1 (ADC).
 * - Modo 1 (Grabando): Almacena el valor del pulso del ADC cada 20ms.
 * - Modo 2 (Reproducción): Reproduce la secuencia grabada usando DMA.
 *
 * Conexiones:
 * - Potenciómetro 1 -> P1.30 (AD0.4)
 * - Servo 1 -> P0.9
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
#define ADC_CH      4   // AD0.4 -> P1.30 -> Potenciómetro 1

// --- Definiciones para Grabación ---
#define PWM_PERIODO_US  20000   // 20ms (Frecuencia de muestreo/reproducción 50Hz)
#define NUM_MUESTRAS    1000    // 1000 muestras @ 50Hz = 20 segundos de grabación

// Variables globales
volatile uint16_t valorADC1 = 0;
volatile uint32_t pulso = 1500;
volatile uint32_t counter = 0;

// --- Variables para Grabación/Reproducción ---
volatile uint8_t systemMode = 0; // 0: Manual, 1: Grabando, 2: Reproduciendo

// --- MODIFICADO ---
// Búfer para 1 solo servo (alineado a 4 bytes)
volatile uint32_t recordedPulses[NUM_MUESTRAS] __attribute__ ((aligned(4)));
volatile uint32_t recordIndex = 0;

// --- Variables DMA ---
// Lista LLI para el DMA (alineada a 16 bytes)
volatile GPDMA_LLI_Type dmaLLI[NUM_MUESTRAS] __attribute__ ((aligned(16)));

//++++++++++++++++++++++++ Config UART (Sin cambios) ++++++++++++++++++++++++
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

    // --- MODIFICADO ---
    // Configurar solo el pin del Servo 1 como salida
    GPIO_SetDir(0, (1<<SERVO_PIN), 1);
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

    // --- MODIFICADO ---
    // Se eliminan configuraciones para ADC_CH5 y ADC_CH2

    ADC_Init(LPC_ADC, 1000);
    ADC_BurstCmd(LPC_ADC, ENABLE);

    // Habilita solo el canal 4
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);

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

        // --- MODIFICADO ---
        // Se eliminan los 'if' para ADC_CH5 y ADC_CH2
    }
    else
    {
        // Si estamos en modo DMA, solo limpia el flag
        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
            ADC_ChannelGetData(LPC_ADC, ADC_CH);

        // --- MODIFICADO ---
        // Se eliminan los 'if' para ADC_CH5 y ADC_CH2
    }
}

//++++++++++++++++++++++++ Timer2 PWM software ++++++++++++++++++++++++
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // Match 0: Periodo de 20ms
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = PWM_PERIODO_US; // 20000
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // Match 1: Pulso para Servo 1
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    // --- MODIFICADO ---
    // Se eliminan configuraciones para Match2 y Match3

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// --- Configuración Timer0 para Grabación (Sin cambios) ---
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
}

// --- Configuración EINT0 (Sin cambios) ---
void ConfEINT(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 11;
    PinCfg.Funcnum = 1; // EINT0
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP; // Asume botón a GND
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    EXTI_Init();
    EXTI_SetMode(EXTI_EINT1, EXTI_MODE_EDGE_SENSITIVE);
    NVIC_EnableIRQ(EINT1_IRQn);
}

// --- Configuración DMA para Reproducción ---
void ConfDMA_Playback(void)
{
    GPDMA_Channel_CFG_Type dmaCfg;
    GPDMA_Init();

    // 1. Construir la Lista Enlazada (LLI)
    for (int i = 0; i < NUM_MUESTRAS; i++)
    {
        // --- MODIFICADO ---
        // Fuente: La muestra i-ésima del búfer de 1 servo
        dmaLLI[i].SrcAddr = (uint32_t)&recordedPulses[i];

        // Destino: SÓLO el registro Match 1 del Timer2 (MR1)
        dmaLLI[i].DstAddr = (uint32_t)&(LPC_TIM2->MR1);

        // Apunta al siguiente item LLI
        dmaLLI[i].NextLLI = (uint32_t)&dmaLLI[i + 1];

        // --- MODIFICADO ---
        // Configuración de control del DMA para esta transferencia
        dmaLLI[i].Control =
            (1 << 0)     |    // TransferSize = 1 word (solo MR1)
            (2 << 18)    |    // Source width = 32 bits
            (2 << 21)    |    // Dest   width = 32 bits
            (0 << 12)    |    // Source burst = 1
            (0 << 15)    |    // Dest   burst = 1
            (1 << 26)    |    // SI = Source increment (Avanza al siguiente 'recordedPulses')
            (0 << 27);        // DI = Dest increment (NO incrementar, escribir siempre en MR1)
    }

    // 2. Hacer que la LLI sea circular (la última apunta a la primera)
    dmaLLI[NUM_MUESTRAS - 1].NextLLI = (uint32_t)&dmaLLI[0];

    // 3. Configurar el Canal 0 del DMA
    dmaCfg.ChannelNum = 0;
    dmaCfg.SrcMemAddr =(uint32_t)recordedPulses;
    dmaCfg.DstMemAddr = 0;
    dmaCfg.TransferSize = 0; // No se usa, definido por LLI
    dmaCfg.TransferWidth = 0; // No se usa, definido por LLI
    dmaCfg.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memoria a Periférico
    dmaCfg.SrcConn = 0;
    dmaCfg.DstConn = GPDMA_CONN_MAT2_0; // <-- usar la conn correcta para Timer2 MR0
    dmaCfg.DMALLI = (uint32_t)&dmaLLI[0]; // Apunta al inicio de la LLI

    GPDMA_Setup(&dmaCfg);
}


// Interrupción Timer2
void TIMER2_IRQHandler(void)
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

        // --- MODIFICADO ---
        // Pone solo el pin del Servo 1 en ALTO
        GPIO_SetValue(0, (1<<SERVO_PIN));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN)); // Pone en BAJO al final del pulso
    }

    // --- MODIFICADO ---
    // Se eliminan los handlers para MR2 y MR3
}

//++++++++++++++++++++++++ Interrupción Timer0 (Grabación) ++++++++++++++++++++++++
void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

        if (systemMode == 1) // Modo Grabación
        {
            if (recordIndex < NUM_MUESTRAS)
            {

                recordedPulses[recordIndex] = pulso;
                recordIndex++;

                if (recordIndex >= NUM_MUESTRAS){
                    recordIndex = 0;
                }
            }
        }
    }
}

//++++++++++++++++++++++++ Interrupción EINT0 (Cambio de Modo) ++++++++++++++++++++++++
void EINT1_IRQHandler(void)
{
    // Simple retardo anti-rebote
    for (volatile int i = 0; i < 1000000; i++);

    EXTI_ClearEXTIFlag(EXTI_EINT1);

    systemMode = (systemMode + 1) % 3; // Cicla 0 -> 1 -> 2 -> 0

    if (systemMode == 0) // Manual
    {
        GPDMA_ChannelCmd(0, DISABLE); // Detiene DMA
        TIM_Cmd(LPC_TIM0, DISABLE);   // Detiene Timer0 (grabación)
        NVIC_EnableIRQ(ADC_IRQn);     // Habilita control por ADC
    }
    else if (systemMode == 1) // Grabando
    {
    	GPIO_SetValue(0,1<<22); // Opcional para debug
        GPDMA_ChannelCmd(0, DISABLE); // Detiene DMA
        recordIndex = 0;              // Reinicia índice de grabación
        NVIC_EnableIRQ(ADC_IRQn);     // Mantiene control por ADC
        TIM_Cmd(LPC_TIM0, ENABLE);    // Inicia Timer0 (tick de grabación)
    }
    else // systemMode == 2: Reproduciendo
    {
    	GPIO_ClearValue(0,1<<22); // Opcional para debug
        TIM_Cmd(LPC_TIM0, DISABLE);   // Detiene Timer0 (grabación)
        NVIC_DisableIRQ(ADC_IRQn);    // Deshabilita control por ADC
        GPDMA_ChannelCmd(0, ENABLE);  // Inicia DMA
    }
}


// Main
int main(void)
{

    GPIO_SetDir(0,1<<22,1);
    GPIO_SetDir(0,1<<22,1);
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
