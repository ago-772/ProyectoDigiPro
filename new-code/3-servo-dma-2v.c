/*
 * Proyecto: Control de 3 servomotores con 3 potenciómetros - LPC1769
 *
 * Descripción:
 * Control de tres servos mediante tres potenciómetros usando ADC en modo burst
 * y generación de PWM por software con el Timer2.
 *
 * MODIFICACIONES:
 * 1- USA EL TIMER0 para tomar los pulsos (muestras) de los ultimos 10 segundos
 * y almacenarlos en un buffer circular (100 muestras @ 10Hz).
 * 2- USA EL GPDMA (Canal 0) para reproducir (playback) los pulsos guardados.
 * - El DMA es M2P (Memory-to-Peripheral).
 * - La fuente (M) es el buffer 'memoriaPulsos'.
 * - El destino (P) son los registros Match del Timer2 (MR1, MR2, MR3).
 * - El trigger (disparo) es el TIMER1 Match 0 (configurado a 10Hz).
 * - Se usa LLI (Linked List Items) para que el DMA funcione en un bucle
 * infinito sobre el buffer de 100 muestras.
 * 3- USA EINT0 (P2.10) para cambiar entre modo ADC (control por potes) y
 * modo PLAYBACK (reproducción en bucle por DMA).
 *
 * Conexiones:
 * - Potenciómetro 1 → P1.30 (AD0.4)
 * - Potenciómetro 2 → P1.31 (AD0.5)
 * - Potenciómetro 3 → P0.25 (AD0.2)
 * - Servo 1 → P0.9
 * - Servo 2 → P0.8
 * - Servo 3 → P0.7
 * - Pulsador (modo) → P2.10 (EINT0)
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpdma.h" // Agregado para DMA
#include <string.h>
#include <stdint.h>
#include <stdio.h>

// Definiciones
#define SERVO_PIN   9   // P0.9  -> Servo 1
#define SERVO_PIN2  8   // P0.8  -> Servo 2
#define SERVO_PIN3  7   // P0.7  -> Servo 3

#define ADC_CH      4   // AD0.4 -> P1.30 -> Potenciómetro 1
#define ADC_CH5     5   // AD0.5 -> P1.31 -> Potenciómetro 2
#define ADC_CH2     2   // AD0.2 -> P0.25 -> Potenciómetro 3

#define TAM_MEMORIA 100 // 100 muestras @ 10Hz = 10 segundos

// Variables globales
volatile uint16_t valorADC1 = 0;
volatile uint16_t valorADC2 = 0;
volatile uint16_t valorADC3 = 0;

volatile uint32_t pulso = 1500;
volatile uint32_t pulso2 = 1500;
volatile uint32_t pulso3 = 1500;

volatile uint32_t counter = 0;

// Nuevas variables globales
volatile uint32_t memoriaPulsos[TAM_MEMORIA][3];
volatile uint32_t write_idx = 0;
// volatile uint32_t read_idx = 0; // No es necesario, el DMA usa LLI
volatile int playback_mode = 0; // 0 = ADC, 1 = Playback

// Lista LLI para el playback circular del DMA
// Debe estar alineada, declararla como static global suele ser suficiente.
volatile GPDMA_LLI_Type lli_playback[TAM_MEMORIA];


// Config UART
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

// Configuración GPIO
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

// Configuración ADC
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
    NVIC_EnableIRQ(ADC_IRQn);
}

// Interrupción ADC
void ADC_IRQHandler(void)
{
    // La EINT0_IRQHandler habilita/deshabilita esta IRQ

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
}

// Timer2 PWM software
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MR0 → periodo 20 ms
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // MR1
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    // MR2
    TIM_MATCHCFG_Type match2 = match1;
    match2.MatchChannel = 2;
    TIM_ConfigMatch(LPC_TIM2, &match2);

    // MR3
    TIM_MATCHCFG_Type match3 = match1;
    match3.MatchChannel = 3;
    TIM_ConfigMatch(LPC_TIM2, &match3);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}


// Interrupción Timer2
void TIMER2_IRQHandler(void)
{
    // MATCH0 → reinicia ciclo PWM
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        counter++;

        if (counter >= 50)
        {
            char mensaje[128];

            // Los valores 'pulso', 'pulso2', 'pulso3' son actualizados
            // por el ADC_IRQHandler o por el DMA (indirectamente)
            uint32_t p1 = LPC_TIM2->MR1;
            uint32_t p2 = LPC_TIM2->MR2;
            uint32_t p3 = LPC_TIM2->MR3;

            if(playback_mode) {
                sprintf(mensaje, "MODO PLAYBACK (DMA): p1=%u p2=%u p3=%u\r\n", p1, p2, p3);
            } else {
                sprintf(mensaje, "MODO ADC: p1=%u p2=%u p3=%u\r\n", p1, p2, p3);
            }

            UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)mensaje,strlen(mensaje),
                      BLOCKING);
            counter = 0;
        }

        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);

        GPIO_SetValue(0, (1<<SERVO_PIN));
        GPIO_SetValue(0, (1<<SERVO_PIN2));
        GPIO_SetValue(0, (1<<SERVO_PIN3));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN));
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

// Timer0 para muestreo/grabación
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1; // Timer corre a 1MHz (1us)
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type matchCfg;
    matchCfg.MatchChannel = 0;
    matchCfg.IntOnMatch = ENABLE;
    matchCfg.ResetOnMatch = ENABLE; // Se resetea cada 100ms
    matchCfg.StopOnMatch = DISABLE;
    matchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    matchCfg.MatchValue = 100000; // 100,000 us = 100ms (10Hz)
    TIM_ConfigMatch(LPC_TIM0, &matchCfg);

    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE); // Inicia el muestreo inmediatamente
}

// Interrupción Timer0 (Grabación)
void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT))
    {
        // Solo graba si no estamos en modo playback
        if (!playback_mode)
        {
            memoriaPulsos[write_idx][0] = pulso;
            memoriaPulsos[write_idx][1] = pulso2;
            memoriaPulsos[write_idx][2] = pulso3;

            write_idx = (write_idx + 1) % TAM_MEMORIA; // Buffer circular
        }

        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
    }
}

// Timer1 para trigger del DMA
void ConfTIMER1(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type matchCfg;
    matchCfg.MatchChannel = 0;
    matchCfg.IntOnMatch = DISABLE;   // NO queremos interrupción
    matchCfg.ResetOnMatch = ENABLE;  // SÍ queremos que se resetee (10Hz)
    matchCfg.StopOnMatch = DISABLE;
    matchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    matchCfg.MatchValue = 100000; // 100ms (10Hz), igual que la grabación
    TIM_ConfigMatch(LPC_TIM1, &matchCfg);

    NVIC_DisableIRQ(TIMER1_IRQn); // Nos aseguramos que la IRQ esté deshabilitada
    // NO se inicia con TIM_Cmd, lo controla la EINT0
}

// Configuración del DMA para Playback
// Configuración del DMA para Playback (loop infinito con LLI)
void ConfDMA(void)
{
    uint32_t i;

    // Construcción de LLI circular
    for (i = 0; i < TAM_MEMORIA; i++)
    {
        lli_playback[i].SrcAddr  = (uint32_t)&memoriaPulsos[i][0];   // pulso1
        lli_playback[i].DstAddr  = (uint32_t)&LPC_TIM2->MR1;         // MR1 siempre
        lli_playback[i].NextLLI  = (uint32_t)&lli_playback[(i+1) % TAM_MEMORIA];

        lli_playback[i].Control =
              GPDMA_DMACCxControl_TransferSize(1)          // Solo 1 WORD por transferencia
            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)
            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)
            | GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD)
            | GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD)
            | GPDMA_DMACCxControl_SI                       // Solo incrementa la fuente
            | 0;                                            // NO incrementa destino
    }

    GPDMA_Init();

    // Configuración del canal DMA0
    GPDMA_Channel_CFG_Type dmaCfg;
    dmaCfg.ChannelNum    = 0;
    dmaCfg.TransferSize  = 1;
    dmaCfg.TransferType  = GPDMA_TRANSFERTYPE_M2P;          // Memoria → Periférico
    dmaCfg.SrcMemAddr    = (uint32_t)&memoriaPulsos[0][0];
    dmaCfg.DstMemAddr    = (uint32_t)&LPC_TIM2->MR1;
    dmaCfg.SrcConn       = 0;
    dmaCfg.DstConn       = GPDMA_CONN_MAT0_0;         
    dmaCfg.DMALLI        = (uint32_t)&lli_playback[0];

    GPDMA_Setup(&dmaCfg);
}



// Interrupción Externa EINT0
void ConfEINT(void)
{
    // Configurar P2.10 como EINT0 (Función 1)
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP; // Asume pulsador a GND
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    // Configurar la interrupción
    EXTI_InitTypeDef extiCfg;
    extiCfg.EXTI_Line = EXTI_EINT0;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE; // Flanco de bajada
    EXTI_Config(&extiCfg);

    EXTI_ClearEXTIFlag(EXTI_EINT0); // Limpiar flag
    NVIC_EnableIRQ(EINT0_IRQn);     // Habilitar en el NVIC
}

// Handler de EINT0 (Cambio de modo)
void EINT0_IRQHandler(void)
{
    playback_mode = !playback_mode; // Cambia de modo (0 -> 1, 1 -> 0)

    if (playback_mode)
    {
        // Entrando a MODO PLAYBACK (DMA)
        NVIC_DisableIRQ(ADC_IRQn);      // Deshabilita interrupción del ADC

        GPDMA_ChannelCmd(0, ENABLE);    // Habilitar Canal 0 DMA
        TIM_Cmd(LPC_TIM1, ENABLE);      // Habilita Timer1 (pacer para el DMA)
    }
    else
    {
        // Entrando a MODO ADC
        TIM_Cmd(LPC_TIM1, DISABLE);     //deshabilita timer1
        GPDMA_ChannelCmd(0, DISABLE);   // Deshabilitar Canal 0 DMA

        NVIC_EnableIRQ(ADC_IRQn);       // Habilita interrupción del ADC
    }

    EXTI_ClearEXTIFlag(EXTI_EINT0); // Limpiar flag de interrupción
}


// Main
int main(void)
{
    ConfGPIO();
    ConfADC();
    ConfTIMER2();
    configGPIO_UART();
    configUART();
    
    ConfTIMER0(); // Inicia la grabación
    ConfTIMER1(); // Configura el pacer/trigger del DMA
    ConfDMA();    // Configura el DMA y la LLI
    ConfEINT();   // Configura el botón de cambio de modo

    while (1)
    {
       
    }
}

