/*
 * Proyecto: Control de 1 servomotor con 1 potenciómetro - LPC1769
 * Autor: [Tu nombre]
 *
 * Descripción:
 * Control de un servo mediante un potenciómetro usando ADC.
 * Implementa 3 modos conmutados por EINT0 (P2.10):
 * 1. MANUAL: Controla el servo con el potenciómetro.
 * 2. GRABACIÓN: (Automático junto con MODO MANUAL) Timer0 guarda los
 * valores del pulso en un buffer circular cada 100ms (10 seg total).
 * 3. REPRODUCCIÓN: DMA transfiere el buffer circular al Match Register
 * del Timer2, reproduciendo el movimiento en bucle.
 *
 * Conexiones:
 * - Potenciómetro 1 → P1.30 (AD0.4)
 * - Servo 1 → P0.9
 * - Pulsador EINT0 → P2.10 (con pull-up, interrumpe por flanco de bajada)
 * - GND común y VCC 5V (para servo)
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_gpdma.h" // Necesario para DMA
#include "lpc17xx_exti.h"  // Necesario para Interrupción Externa

// ---------------------------------------------------------------------------
// Definiciones
// ---------------------------------------------------------------------------
#define SERVO_PIN 9 // P0.9  -> Servo 1
#define ADC_CH 4    // AD0.4 -> P1.30 -> Potenciómetro 1
#define EINT_PIN 10 // P2.10 -> EINT0

#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

// Buffer para 10 segundos de grabación, muestreando cada 100ms
#define BUFFER_SAMPLE_MS 100
#define BUFFER_DURATION_S 10
#define BUFFER_SIZE ( (BUFFER_DURATION_S * 1000) / BUFFER_SAMPLE_MS ) // 100 muestras

// ---------------------------------------------------------------------------
// Variables globales
// ---------------------------------------------------------------------------
volatile uint16_t valorADC1 = 0;
volatile uint32_t pulso = 1500; // Valor de pulso actual (controlado por ADC)

// Buffer circular para grabación
volatile uint32_t pulsoBuffer[BUFFER_SIZE];
volatile uint32_t bufferIndex = 0;

// Variables de estado y DMA
volatile uint8_t modoPlayback = 0; // 0 = Manual/Record, 1 = Playback
GPDMA_Channel_CFG_Type dmaCfg;
GPDMA_LLI_Type dmaLLI;

// ---------------------------------------------------------------------------
// Configuración GPIO
// ---------------------------------------------------------------------------
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0; // GPIO
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    // Servo 1 (P0.9)
    PinCfg.Pinnum = SERVO_PIN;
    PINSEL_ConfigPin(&PinCfg);

    // Configurar pin como salida
    GPIO_SetDir(0, (1 << SERVO_PIN), 1);
}

// ---------------------------------------------------------------------------
// Configuración ADC
// ---------------------------------------------------------------------------
void ConfADC(void)
{
    PINSEL_CFG_Type PinCfg;

    // Potenciómetro 1 -> P1.30 (AD0.4)
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // Inicializar ADC
    ADC_Init(LPC_ADC, 1000); // 1MHz, muy bajo, pero funciona para el ejemplo
    ADC_BurstCmd(LPC_ADC, ENABLE);

    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
    NVIC_EnableIRQ(ADC_IRQn);

    GPIO_SetValue(0,1<<22); 						// LED
}

// ---------------------------------------------------------------------------
// Interrupción ADC
// ---------------------------------------------------------------------------
void ADC_IRQHandler(void)
{
    // --- Canal AD0.4 (P1.30) ---
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        // Mapeo lineal de 0-4095 (ADC) a 1000-2000 (Servo)
        pulso = SERVO_MIN_US + ((uint32_t)valorADC1 * (SERVO_MAX_US - SERVO_MIN_US) / 4095);

        // Actualiza el Match solo si no estamos en Playback (para no interferir con DMA)
        // (Nota: esta IRQ estará deshabilitada en modo Playback de todos modos)
        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }
}

// ---------------------------------------------------------------------------
// Configuración Timer2 (PWM software)
// ---------------------------------------------------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1; // 1 tick = 1 microsegundo
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // --- MATCH0: periodo 20 ms (20000 us) ---
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING; // Se usará para disparar DMA
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // --- MATCH1: Servo 1 ---
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500; // Valor inicial
    TIM_ConfigMatch(LPC_TIM2, &match1);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// ---------------------------------------------------------------------------
// Interrupción Timer2
// ---------------------------------------------------------------------------
void TIMER2_IRQHandler(void)
{
    // MATCH0 → reinicia ciclo PWM, pone el pin en ALTO
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1 << SERVO_PIN));
    }

    // MATCH1 → fin pulso servo 1, pone el pin en BAJO
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1 << SERVO_PIN));
    }
}

// ---------------------------------------------------------------------------
// Configuración Timer0 (Muestreo para grabación)
// ---------------------------------------------------------------------------
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1; // 1us tick
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = BUFFER_SAMPLE_MS * 1000; // Interrumpe cada 100ms
    TIM_ConfigMatch(LPC_TIM0, &match0);

    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

// ---------------------------------------------------------------------------
// Interrupción Timer0 (Grabación en buffer)
// ---------------------------------------------------------------------------
void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

        // Solo graba si NO estamos en modo Playback
        if (modoPlayback == 0)
        {
            pulsoBuffer[bufferIndex] = pulso; // Guarda el valor actual (de la IRQ del ADC)
            bufferIndex++;
            if (bufferIndex >= BUFFER_SIZE)
            {
                bufferIndex = 0; // Buffer circular
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Configuración DMA (Playback)
// ---------------------------------------------------------------------------
void ConfDMA(void)
{
    // Preparar LLI (Linked List Item) para transferencia circular
    dmaLLI.SrcAddr = (uint32_t)	pulsoBuffer;
    dmaLLI.DstAddr = (uint32_t) &(LPC_TIM2->MR1); // Destino: Match Register 1 del Timer 2
    dmaLLI.NextLLI = (uint32_t)	&dmaLLI;          // Bucle: apunta a sí mismo
    dmaLLI.Control =
          (BUFFER_SIZE)                // Bits 0–11: TransferSize
        | (0 << 12)                    // SBSize (ignored for M2P normal DMA)
        | (0 << 15)                    // DBSize (ignored for M2P normal DMA)
        | (2 << 18)                    // SWidth = 2 → 32-bit
        | (2 << 21)                    // DWidth = 2 → 32-bit
        | (1 << 26)                    // SI = 1 → Source Increment enable
        | (0 << 27)                    // DI = 0 → Dest Increment disable (siempre MR1)
        | (1 << 31);                   // I = 1 → Interrupt enable (opcional)

    GPDMA_Init();

    // Configurar Canal 0
    dmaCfg.ChannelNum = 0;
    dmaCfg.SrcMemAddr = (uint32_t)pulsoBuffer; // Fuente
    dmaCfg.DstMemAddr = 0; // No se usa para M2P
    dmaCfg.TransferSize = BUFFER_SIZE;
    dmaCfg.TransferWidth = GPDMA_WIDTH_WORD;
    dmaCfg.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memoria a Periférico
    dmaCfg.SrcConn = 0;
    dmaCfg.DstConn = GPDMA_CONN_MAT2_0; // Disparado por Timer2 Match 0
    dmaCfg.DMALLI = (uint32_t)&dmaLLI;       // Usar LLI para bucle

    GPDMA_Setup(&dmaCfg);

    // NO habilitar el canal todavía. Se habilita/deshabilita con EINT.
    // GPDMA_ChannelCmd(0, ENABLE);
}

// ---------------------------------------------------------------------------
// Configuración EINT (Cambio de modo)
// ---------------------------------------------------------------------------
void ConfEINT(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = EINT_PIN; // P2.10
    PinCfg.Funcnum = 1;       // EINT0
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP; // Asume pulsador a GND
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    GPIO_SetDir(2, (1 << EINT_PIN), 0); // P2.10 como entrada

    EXTI_InitTypeDef extiCfg;
    extiCfg.EXTI_Line = EXTI_EINT0;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE; // Interrumpe al presionar
    EXTI_Config(&extiCfg);

    EXTI_ClearEXTIFlag(EXTI_EINT0);
    NVIC_EnableIRQ(EINT0_IRQn);
}

// ---------------------------------------------------------------------------
// Interrupción EINT0 (Manejador de estado)
// ---------------------------------------------------------------------------
void EINT0_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT0);

    if (modoPlayback == 0)
    {
        // --- Cambiar a modo PLAYBACK ---
        modoPlayback = 1;

        // Deshabilitar ADC_IRQ para que no actualice MR1 manualmente
        NVIC_DisableIRQ(ADC_IRQn);
        ADC_IntConfig(LPC_ADC, ADC_CH, DISABLE);

        // Habilitar canal DMA (inicia el playback)
        GPDMA_ChannelCmd(0, ENABLE);
        GPIO_ClearValue(0,1<<22); 						// LED
    }
    else
    {
        // --- Cambiar a modo MANUAL/RECORD ---
        modoPlayback = 0;

        // Deshabilitar canal DMA (detiene el playback)
        GPDMA_ChannelCmd(0, DISABLE);

        // Habilitar ADC_IRQ
        ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
        NVIC_EnableIRQ(ADC_IRQn);
        GPIO_SetValue(0,1<<22); 						// LED
    }
}

// ---------------------------------------------------------------------------
// Función principal
// ---------------------------------------------------------------------------
int main(void)
{
    GPIO_SetDir(0,1<<22,1);
    GPIO_SetDir(0,1<<22,1);

    ConfGPIO();   // Configurar pin de servo
    ConfADC();    // Configurar ADC (potenciómetro)
    ConfTIMER2(); // Configurar Timer2 para PWM software
    ConfTIMER0(); // Configurar Timer0 para muestreo de grabación
    ConfDMA();    // Configurar DMA para playback
    ConfEINT();   // Configurar EINT para cambio de modo

    while (1)
    {

    }
}
