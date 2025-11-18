/*
 * Proyecto: Grabadora Servo Optimizada (Sin Timer0 - Doble DMA)
 * MCU: LPC1769
 *
 * Flujo de Datos:
 * 1. MODO MANUAL/REC: ADC (IRQ hace matemática) -> variable 'pulso_actual'.
 * 2. MODO REC (DMA0): variable 'pulso_actual' -> Buffer RAM (Trigger: Timer2 PWM Match).
 * 3. MODO PLAY (DMA1): Buffer RAM -> Timer2 MR1 (Trigger: Timer2 PWM Match).
 *
 * Buffer aumentado a 2000 muestras (aprox 40 segundos a 50Hz).
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_exti.h"

// --- Definiciones ---
#define SERVO_PIN       9    // P0.9
#define LED_REC         22   // P0.22 (Led Grabando)
#define ADC_CH          4    // P1.30

#define PWM_PERIOD_US   20000    // 20ms
#define BUFFER_SIZE     2000     //  2000 muestras

// --- Variables Globales ---
volatile uint32_t pulso_actual = 1500; // Variable puente (limpia)
uint32_t servo_buffer[BUFFER_SIZE];    // Buffer grande en RAM
volatile uint8_t systemMode = 0;       // 0: Manual, 1: Rec, 2: Play

// Estructura para la Lista Enlazada (Solo usada en Playback para bucle infinito)
GPDMA_LLI_Type LLI_Play;

// --- Configuración GPIO ---
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0; // GPIO por defecto (PWM lo sobreescribe luego)
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    // LED
    PinCfg.Pinnum = LED_REC;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<LED_REC), 1);
}

// --- Configuración ADC ---
void ConfADC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3; // AD0.4
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_BurstCmd(LPC_ADC, ENABLE); // Burst mode: conversión continua

    NVIC_EnableIRQ(ADC_IRQn);
}

// --- Configuración Timer2 (PWM y Trigger Maestro) ---
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MATCH 0: Periodo (20ms). También dispara el DMA.
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;      // Necesario para generar el pulso PWM por soft o hard
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = PWM_PERIOD_US;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // MATCH 1: Ancho del Pulso (Servo)
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// --- DMA Config: GRABACIÓN (Simple, sin LLI) ---
void SetupDMA_Record(void)
{
    GPDMA_Channel_CFG_Type GPDMACfg;

    // Transferencia simple P2M (Peripheral to Memory)
    // Aunque la fuente es una variable (&pulso_actual), usamos P2M para sincronizar
    // con el Trigger del Timer2.

    GPDMACfg.ChannelNum 		= 0;
    GPDMACfg.SrcMemAddr 		= (uint32_t)&pulso_actual; // Fuente fija
    GPDMACfg.DstMemAddr 		= (uint32_t)servo_buffer;  // Destino: Array
    GPDMACfg.TransferSize 		= BUFFER_SIZE;
    GPDMACfg.TransferWidth 		= 0;
    GPDMACfg.TransferType 		= GPDMA_TRANSFERTYPE_P2M;
    GPDMACfg.SrcConn 			= GPDMA_CONN_MAT2_0;
    GPDMACfg.DstConn 			= 0;

    GPDMACfg.DMALLI 			= 0; // Sin lista enlazada (lineal)

    GPDMA_Setup(&GPDMACfg);
}

// --- DMA Config: PLAYBACK (Con LLI Circular) ---
void SetupDMA_Play(void)
{
    // Configurar LLI para bucle infinito
    LLI_Play.SrcAddr = (uint32_t)servo_buffer;
    LLI_Play.DstAddr = (uint32_t)&(LPC_TIM2->MR1); // Escribir en registro de PWM
    LLI_Play.NextLLI = (uint32_t)&LLI_Play;        // Se apunta a sí misma (Circular)
    LLI_Play.Control = BUFFER_SIZE
                     | (2 << 18) // Width 32
                     | (2 << 21) // Width 32
                     | (1 << 26) // Source Inc: SI
                     | (0 << 27) // Dest Inc: NO
                     | (0 << 31); // No Int

    GPDMA_Channel_CFG_Type GPDMACfg;
    GPDMACfg.ChannelNum 		= 1;
    GPDMACfg.SrcMemAddr 		= (uint32_t)servo_buffer;
    GPDMACfg.DstMemAddr			= (uint32_t)&(LPC_TIM2->MR1);
    GPDMACfg.TransferSize 		= BUFFER_SIZE;
    GPDMACfg.TransferType 		= GPDMA_TRANSFERTYPE_M2P; // Memory to Peripheral (Timer)

    GPDMACfg.SrcConn 			= 0;
    GPDMACfg.DstConn 			= GPDMA_CONN_MAT2_0; // Trigger: Mismo Timer2 Match 0

    GPDMACfg.DMALLI = (uint32_t)&LLI_Play;

    GPDMA_Setup(&GPDMACfg);
}

// --- Configuración Botón ---
void ConfEINT1(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 11;
    PinCfg.Funcnum = 1;
    PINSEL_ConfigPin(&PinCfg);

    EXTI_InitTypeDef ExtiCfg;
    ExtiCfg.EXTI_Line = EXTI_EINT1;
    ExtiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    ExtiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&ExtiCfg);
    NVIC_EnableIRQ(EINT1_IRQn);
}

// --- IRQ Botón: Lógica de Estados ---
void EINT1_IRQHandler(void)
{
    for(int i=0; i<1000000; i++); // Delay
    EXTI_ClearEXTIFlag(EXTI_EINT1);

    systemMode = (systemMode + 1) % 3;

    // Apagar canales DMA previos
    GPDMA_ChannelCmd(0, DISABLE);
    GPDMA_ChannelCmd(1, DISABLE);

    if (systemMode == 0) 					// MANUAL
    {
        GPIO_ClearValue(0, 1<<LED_REC);
        NVIC_EnableIRQ(ADC_IRQn); // ADC controla servo
    }
    else if (systemMode == 1) 				// GRABAR
    {
        GPIO_SetValue(0, 1<<LED_REC);
        NVIC_EnableIRQ(ADC_IRQn); // ADC sigue controlando servo (para verlo moverse)

        SetupDMA_Record();        // Configura DMA Lineal
        GPDMA_ChannelCmd(0, ENABLE);
    }
    else 									// PLAY
    {
        GPIO_ClearValue(0, 1<<LED_REC);
        NVIC_DisableIRQ(ADC_IRQn); // ADC Callado

        SetupDMA_Play();          // Configura DMA Circular
        GPDMA_ChannelCmd(1, ENABLE);
    }
}

// --- IRQ ADC: Matemática y Control Manual ---
void ADC_IRQHandler(void)
{
    if (systemMode != 2) // Si no estamos en Play
    {
        uint16_t adcVal = ADC_ChannelGetData(LPC_ADC, ADC_CH);
        uint32_t pulso;

        // Matemática simple para convertir 12-bit a 1000-2000us
        // Hacemos esto aqui porque el DMA no sabe dividir/multiplicar
        if (adcVal < 100) pulso = 1000;
        else if (adcVal > 4000) pulso = 2000;
        else pulso = 1000 + (adcVal * 1000) / 4095;

        // 1. Movemos el servo REAL (feedback visual)
        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);

        // 2. Actualizamos la variable puente
        // El DMA Canal 0 vendrá aquí a buscar el dato cuando el Timer2 lo diga
        pulso_actual = pulso;
    }
}

// --- IRQ Timer2: Generación PWM (Manual) ---
// Esta interrupción se ejecuta cada 20ms y cada vez que termina el pulso high
void TIMER2_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT)) // Inicio del ciclo (20ms)
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1<<SERVO_PIN));

        // Nota: Aquí es donde el hardware dispara internamente el DMA request
        // No necesitamos código extra, el bit MCR lo hace solo.
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) // Fin del pulso (1ms-2ms)
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN));
    }
}

int main(void)
{
    GPDMA_Init();
    ConfGPIO();
    ConfADC();
    ConfTIMER2(); // Inicia el PWM
    ConfEINT1();

    while(1)
    {
        // CPU dormida, todo ocurre por interrupciones y DMA
        __WFI();
    }
}
