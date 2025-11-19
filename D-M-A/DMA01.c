/*
 * Proyecto: Servo + ADC + DMA + EINT1
 * MCU: LPC1769
 * Review: Gemini AI
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_exti.h"

// ---------------------------------------------------------------------------
// Definiciones
// ---------------------------------------------------------------------------
#define SERVO_PIN   9           // P0.9
#define ADC_CH      4           // AD0.4 (P1.30)
#define BUFFER_SIZE 256         // Tam. buffer DMA

// ---------------------------------------------------------------------------
// Variables globales
// ---------------------------------------------------------------------------
volatile uint16_t valorADC = 0;
volatile uint32_t pulso = 1500;

uint32_t adcBuffer[BUFFER_SIZE];

volatile uint32_t modo = 0;      // 0 = manual, 1 = grabar, 2 = reproducir
volatile uint32_t indicePlay = 0;

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    PinCfg.Pinnum = SERVO_PIN;
    PINSEL_ConfigPin(&PinCfg);

    GPIO_SetDir(0, (1<<SERVO_PIN), 1);
}

// ---------------------------------------------------------------------------
// DMA (ADC -> RAM)
// ---------------------------------------------------------------------------
void ConfDMA(void)
{
    GPDMA_Channel_CFG_Type dmaCfg;

    GPDMA_Init();

    dmaCfg.ChannelNum    = 0;
    dmaCfg.SrcMemAddr    = 0; // Ignorado para perifericos
    dmaCfg.DstMemAddr    = (uint32_t)adcBuffer; // MODIFICADO: Casteo explícito
    dmaCfg.TransferSize  = BUFFER_SIZE;
    dmaCfg.TransferWidth = GPDMA_WIDTH_WORD;
    dmaCfg.TransferType  = GPDMA_TRANSFERTYPE_P2M;
    dmaCfg.SrcConn       = GPDMA_CONN_ADC;
    dmaCfg.DstConn       = 0;
    dmaCfg.DMALLI        = 0;

    GPDMA_Setup(&dmaCfg);
}

// ---------------------------------------------------------------------------
// ADC
// ---------------------------------------------------------------------------
void ConfADC(void)
{
    PINSEL_CFG_Type PinCfg;

    PinCfg.Portnum = 1;
    PinCfg.Pinnum  = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // MODIFICADO: Aumentada frecuencia a 200kHz para estabilidad del ADC
    ADC_Init(LPC_ADC, 200000);

    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);

    // MODIFICADO: Habilitar interrupcion global del ADC aqui para asegurar orden
    NVIC_EnableIRQ(ADC_IRQn);

    // Iniciar en modo Burst (Continuo)
    ADC_BurstCmd(LPC_ADC, ENABLE);
}

// ---------------------------------------------------------------------------
// IRQ ADC -- Muestreo Manual
// ---------------------------------------------------------------------------
void ADC_IRQHandler(void)
{
    // Verificamos si el canal 4 terminó la conversión
    // Nota: Si el DMA está activo, podría robarse la lectura y limpiar el flag
    // antes de llegar aqui, pero intentamos leer igual para mantener el servo vivo.
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        // FORMULA EXACTA SOLICITADA
        if (valorADC >= 1948 && valorADC < 2147)
            pulso = 1500;
        else if (valorADC < 1948)
            pulso = 1000 + (uint32_t)(((float)valorADC / 1947.0f) * 500.0f);
        else
            pulso = 1500 + (uint32_t)(((float)(valorADC - 2147) / 1948.0f) * 500.0f);

        // Solo actualizamos el Timer si NO estamos en modo reproducir (Modo 2)
        if (modo != 2)
        {
            TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
        }
    }
}

// ---------------------------------------------------------------------------
// TIMER2 -- PWM software para servo
// ---------------------------------------------------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MATCH0 -> 20ms (Periodo)
    TIM_MATCHCFG_Type m0;
    m0.MatchChannel = 0;
    m0.IntOnMatch   = ENABLE;
    m0.ResetOnMatch = ENABLE;
    m0.StopOnMatch  = DISABLE;
    m0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m0.MatchValue   = 20000;
    TIM_ConfigMatch(LPC_TIM2, &m0);

    // MATCH1 -> Ancho de pulso (Servo)
    TIM_MATCHCFG_Type m1;
    m1.MatchChannel = 1;
    m1.IntOnMatch   = ENABLE;
    m1.ResetOnMatch = DISABLE;
    m1.StopOnMatch  = DISABLE;
    m1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m1.MatchValue   = 1500;
    TIM_ConfigMatch(LPC_TIM2, &m1);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

void TIMER2_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1<<SERVO_PIN)); // Inicio del pulso
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN)); // Fin del pulso
    }
}

// ---------------------------------------------------------------------------
// EINT1 -- Configuración
// ---------------------------------------------------------------------------
void ConfEINT1(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 11;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    EXTI_InitTypeDef extiCfg;
    extiCfg.EXTI_Line = EXTI_EINT1;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&extiCfg);

    EXTI_ClearEXTIFlag(EXTI_EINT1);
    NVIC_EnableIRQ(EINT1_IRQn);
}

// ---------------------------------------------------------------------------
// Handler EINT1 -- Máquina de estados
// ---------------------------------------------------------------------------
void EINT1_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT1);

    // Pequeño delay anti-rebote simple (bloqueante, solo para pruebas)
    for(int i=0; i<100000; i++);

    if (modo == 0)
    {
        // ----- MODO 1: GRABAR -----
        modo = 1;
        indicePlay = 0;

        // Aseguramos que el Timer de reproducción esté apagado
        TIM_Cmd(LPC_TIM0, DISABLE);

        // Reconfigurar y Habilitar DMA para llenar el buffer
        ConfDMA();
        GPDMA_ChannelCmd(0, ENABLE);
        GPIO_SetValue(0,1<<22);
    }
    else if (modo == 1)
    {
        // ----- MODO 2: REPRODUCIR -----
        modo = 2;
        indicePlay = 0;

        // Detener DMA (ya no grabamos)
        GPDMA_ChannelCmd(0, DISABLE);

        // Iniciar Timer0 para leer buffer y mover servo
        TIM_Cmd(LPC_TIM0, ENABLE);
        GPIO_ClearValue(0,1<<22);
    }
    else if (modo == 2)
    {
        // ----- MODIFICADO: VUELTA A MODO 0 (MANUAL) -----
        modo = 0;

        // Apagar Timer de reproducción
        TIM_Cmd(LPC_TIM0, DISABLE);

        // DMA apagado
        GPDMA_ChannelCmd(0, DISABLE);
        GPIO_ClearValue(0,1<<22);
    }
}

// ---------------------------------------------------------------------------
// TIMER0 -- Timer de Reproducción
// ---------------------------------------------------------------------------
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;   // 1us resolution

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    // MATCH0 -> interrupción cada 20ms (misma tasa que el servo)
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.MatchValue   = 20000;
    match0.IntOnMatch   = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch  = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

    TIM_ConfigMatch(LPC_TIM0, &match0);

    NVIC_EnableIRQ(TIMER0_IRQn);
}

void TIMER0_IRQHandler(void)
{
    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

    // MODIFICADO: El DMA guarda el registro de datos COMPLETO de 32 bits.
    // Los bits 4 a 15 contienen el valor de 12 bits del ADC.
    // Debemos desplazar y enmascarar.
    uint32_t rawData = adcBuffer[indicePlay];
    uint32_t valorGrabado = (rawData >> 4) & 0xFFF;

    uint32_t pulsoPlay;

    // ---- FORMULA EXACTA ----
    if (valorGrabado >= 1948 && valorGrabado < 2147)
        pulsoPlay = 1500;
    else if (valorGrabado < 1948)
        pulsoPlay = 1000 + (uint32_t)(((float)valorGrabado / 1947.0f) * 500.0f);
    else
        pulsoPlay = 1500 + (uint32_t)(((float)(valorGrabado - 2147) / 1948.0f) * 500.0f);

    // Actualizar servo (Timer2) con el valor leido del buffer
    TIM_UpdateMatchValue(LPC_TIM2, 1, pulsoPlay);

    // Avanzar índice
    indicePlay++;
    if (indicePlay >= BUFFER_SIZE)
    {
        indicePlay = 0; // Bucle infinito de reproducción
    }
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(void)
{
	GPIO_SetDir(0,1<<22,1);

    ConfGPIO();
    ConfTIMER2();
    ConfTIMER0(); // Configuramos Timer0 pero no lo habilitamos aun
    ConfDMA();
    ConfADC();    // ADC al final
    ConfEINT1();

    while (1)
    {
        // Modo 0: ADC_IRQ mueve Servo
        // Modo 1: DMA llena Buffer (muy rápido)
        // Modo 2: TIMER0_IRQ lee Buffer y mueve Servo
    }
}