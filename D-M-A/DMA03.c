/*
 * Proyecto: Servo + ADC + DMA + EINT1
 * MCU: LPC1769
 * Placa: LPCXpresso (LED en P0.22)
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
#define SERVO_PIN   9           // P0.9 (Salida PWM Servo)
#define LED_PIN     22          // P0.22 (LED integrado LPCXpresso)
#define ADC_CH      4           // AD0.4 (P1.30)
#define BUFFER_SIZE 256         // Tamaño del buffer de grabación

// ---------------------------------------------------------------------------
// Variables globales
// ---------------------------------------------------------------------------
volatile uint16_t valorADC = 0;
volatile uint32_t pulso = 1500;

// Buffer donde el DMA guardará los datos crudos del registro del ADC
uint32_t adcBuffer[BUFFER_SIZE];

// Modo: 0 = GRABAR (Manual), 1 = REPRODUCIR (Automático)
volatile uint32_t modo = 0;      
volatile uint32_t indicePlay = 0;

// ---------------------------------------------------------------------------
// Declaración de Funciones
// ---------------------------------------------------------------------------
void ConfGPIO(void);
void ConfDMA(void);
void ConfADC(void);
void ConfTIMER2(void);
void ConfTIMER0(void);
void ConfEINT1(void);

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

    // 1. Configurar PIN del Servo (P0.9)
    PinCfg.Pinnum = SERVO_PIN;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<SERVO_PIN), 1);

    // 2. MODIFICADO: Configurar PIN del LED (P0.22)
    PinCfg.Pinnum = LED_PIN;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<LED_PIN), 1);
}

// ---------------------------------------------------------------------------
// DMA (ADC -> RAM)
// ---------------------------------------------------------------------------
void ConfDMA(void)
{
    GPDMA_Channel_CFG_Type dmaCfg;

    // Inicializar si no está activo
    GPDMA_Init();

    dmaCfg.ChannelNum    = 0;
    dmaCfg.SrcMemAddr    = 0; // Ignorado para periféricos
    dmaCfg.DstMemAddr    = (uint32_t)adcBuffer; 
    dmaCfg.TransferSize  = BUFFER_SIZE;
    dmaCfg.TransferWidth = GPDMA_WIDTH_WORD; // 32 bits (registro completo ADC)
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
    PinCfg.Funcnum = 3; // AD0.4
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // 200kHz para estabilidad
    ADC_Init(LPC_ADC, 200000); 
    
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    
    NVIC_EnableIRQ(ADC_IRQn);

    // Modo Burst (Conversión continua) para alimentar al DMA
    ADC_BurstCmd(LPC_ADC, ENABLE);
}

// ---------------------------------------------------------------------------
// IRQ ADC — Control Manual (Modo 0)
// ---------------------------------------------------------------------------
void ADC_IRQHandler(void)
{
    // Aunque el DMA robe los datos, leemos para actualizar el servo en tiempo real
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        // FÓRMULA EXACTA
        if (valorADC >= 1948 && valorADC < 2147)
            pulso = 1500;
        else if (valorADC < 1948)
            pulso = 1000 + (uint32_t)(((float)valorADC / 1947.0f) * 500.0f);
        else
            pulso = 1500 + (uint32_t)(((float)(valorADC - 2147) / 1948.0f) * 500.0f);

        // MODIFICADO: Solo actualizamos el servo aquí si estamos GRABANDO (Modo 0)
        if (modo == 0) 
        {
            TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
        }
    }
}

// ---------------------------------------------------------------------------
// TIMER2 — Generación PWM para Servo (20ms periodo)
// ---------------------------------------------------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MATCH0 -> 20ms (Periodo total)
    TIM_MATCHCFG_Type m0;
    m0.MatchChannel = 0;
    m0.IntOnMatch   = ENABLE;
    m0.ResetOnMatch = ENABLE;
    m0.StopOnMatch  = DISABLE;
    m0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m0.MatchValue   = 20000;
    TIM_ConfigMatch(LPC_TIM2, &m0);

    // MATCH1 -> Ancho de pulso (Posición Servo)
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
        GPIO_SetValue(0, (1<<SERVO_PIN)); // Pin ALTO al inicio del periodo
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN)); // Pin BAJO cuando se cumple el ancho de pulso
    }
}

// ---------------------------------------------------------------------------
// TIMER0 — Reproductor (Modo 1)
// ---------------------------------------------------------------------------
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;   

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    // MATCH0 -> Interrupción cada 20ms para actualizar el servo
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

    // Leer del buffer
    uint32_t rawData = adcBuffer[indicePlay];
    
    // MODIFICADO: El DMA guarda 32 bits. El valor de 12 bits está desplazado 4 bits.
    uint32_t valorGrabado = (rawData >> 4) & 0xFFF; 

    uint32_t pulsoPlay;

    // ---- FÓRMULA EXACTA (Aplicada al valor reproducido) ----
    if (valorGrabado >= 1948 && valorGrabado < 2147)
        pulsoPlay = 1500;
    else if (valorGrabado < 1948)
        pulsoPlay = 1000 + (uint32_t)(((float)valorGrabado / 1947.0f) * 500.0f);
    else
        pulsoPlay = 1500 + (uint32_t)(((float)(valorGrabado - 2147) / 1948.0f) * 500.0f);

    // Actualizar servo
    TIM_UpdateMatchValue(LPC_TIM2, 1, pulsoPlay);

    // Avanzar índice
    indicePlay++;
    if (indicePlay >= BUFFER_SIZE)
    {
        indicePlay = 0; // Bucle infinito
    }
}

// ---------------------------------------------------------------------------
// EINT1 — Botón de Cambio de Modo
// ---------------------------------------------------------------------------
void ConfEINT1(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 11;
    PinCfg.Funcnum = 1; // EINT1
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

void EINT1_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT1); 
    
    // Delay simple anti-rebote
    for(volatile int i=0; i<100000; i++); 

    if (modo == 1) 
    {
        // -------------------------------------------
        // Estaba en REPRODUCIR (1) -> Pasa a GRABAR (0)
        // -------------------------------------------
        modo = 0;

        // 1. Apagar Timer Reproductor
        TIM_Cmd(LPC_TIM0, DISABLE);

        // 2. Reiniciar DMA para grabar nuevo buffer
        ConfDMA(); 
        GPDMA_ChannelCmd(0, ENABLE);

        // 3. MODIFICADO: Encender LED (P0.22)
        GPIO_SetValue(0, (1<<LED_PIN));
    }
    else 
    {
        // -------------------------------------------
        // Estaba en GRABAR (0) -> Pasa a REPRODUCIR (1)
        // -------------------------------------------
        modo = 1;
        indicePlay = 0;

        // 1. Detener DMA
        GPDMA_ChannelCmd(0, DISABLE);

        // 2. Iniciar Timer Reproductor
        TIM_Cmd(LPC_TIM0, ENABLE);

        // 3. MODIFICADO: Apagar LED (P0.22)
        GPIO_ClearValue(0, (1<<LED_PIN));
    }
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(void)
{
    ConfGPIO();
    
    GPIO_SetDir(0,1<<22,1);


    ConfTIMER2(); // PWM Servo
    ConfTIMER0(); // Reproductor (inicia apagado)
    ConfDMA();    // Prepara buffer
    ConfADC();    // Inicia conversión
    ConfEINT1();  // Botón

    while (1)
    {
        // Bucle vacío.
        // Todo ocurre en ADC_IRQHandler (Modo 0)
        // O en TIMER0_IRQHandler (Modo 1)
        // EINT1_IRQHandler cambia entre ellos.
    }
}