/*
 * Proyecto: Servo + ADC + DMA + EINT1 + DAC 
 * MCU: LPC1769
 * Placa: LPCXpresso (LED en P0.22)
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"  
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
void ConfDAC(void);  
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

    // 2. Configurar PIN del LED (P0.22)
    PinCfg.Pinnum = LED_PIN;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<LED_PIN), 1);
}

// ---------------------------------------------------------------------------
// DAC (Salida Analógica P0.26)
// ---------------------------------------------------------------------------
void ConfDAC(void)
{
    PINSEL_CFG_Type PinCfg;
    
    // P0.26 es AOUT 
    PinCfg.Portnum = 0;
    PinCfg.Pinnum  = 26;
    PinCfg.Funcnum = 2; // Función 2 es AOUT
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP; // Bias interno
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    
    PINSEL_ConfigPin(&PinCfg);

    DAC_Init(LPC_DAC);
}

// ---------------------------------------------------------------------------
// DMA (ADC -> RAM)
// ---------------------------------------------------------------------------
void ConfDMA(void)
{
    GPDMA_Channel_CFG_Type dmaCfg;

    GPDMA_Init();

    dmaCfg.ChannelNum    = 0;
    dmaCfg.SrcMemAddr    = 0; 
    dmaCfg.DstMemAddr    = (uint32_t)adcBuffer; 
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
    PinCfg.Funcnum = 3; // AD0.4
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000); 
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    NVIC_EnableIRQ(ADC_IRQn);
    ADC_BurstCmd(LPC_ADC, ENABLE);
}

// ---------------------------------------------------------------------------
// IRQ ADC — Control Manual (Modo 0)
// ---------------------------------------------------------------------------
void ADC_IRQHandler(void)
{
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH); // 12 bits

        // FÓRMULA EXACTA
        if (valorADC >= 1948 && valorADC < 2147)
            pulso = 1500;
        else if (valorADC < 1948)
            pulso = 1000 + (uint32_t)(((float)valorADC / 1947.0f) * 500.0f);
        else
            pulso = 1500 + (uint32_t)(((float)(valorADC - 2147) / 1948.0f) * 500.0f);

        // Si estamos GRABANDO (Modo 0)
        if (modo == 0) 
        {
            TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
            
            // OPCIONAL: Actualizar DAC también en vivo para ver lo que grabas
            // ADC (12 bit) -> DAC (10 bit) : Desplazamos 2 a la derecha
            DAC_UpdateValue(LPC_DAC, (valorADC >> 2));
        }
    }
}

// ---------------------------------------------------------------------------
// TIMER2 — PWM Servo
// ---------------------------------------------------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type m0;
    m0.MatchChannel = 0;
    m0.IntOnMatch   = ENABLE;
    m0.ResetOnMatch = ENABLE;
    m0.StopOnMatch  = DISABLE;
    m0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m0.MatchValue   = 20000;
    TIM_ConfigMatch(LPC_TIM2, &m0);

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
        GPIO_SetValue(0, (1<<SERVO_PIN)); 
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN)); 
    }
}

// ---------------------------------------------------------------------------
// TIMER0 — Reproductor (Modo 1) + DAC Output
// ---------------------------------------------------------------------------
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;   

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

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

// ----- AQUÍ ESTÁ LA MAGIA DEL DAC EN REPRODUCCIÓN -----
void TIMER0_IRQHandler(void)
{
    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

    // 1. Leer del buffer (Dato crudo de 32 bits del DMA)
    uint32_t rawData = adcBuffer[indicePlay];
    
    // 2. Extraer los 12 bits reales del ADC (Bits 4 a 15)
    uint32_t valorGrabado = (rawData >> 4) & 0xFFF; 

    // 3. Calcular PWM para Servo
    uint32_t pulsoPlay;
    if (valorGrabado >= 1948 && valorGrabado < 2147)
        pulsoPlay = 1500;
    else if (valorGrabado < 1948)
        pulsoPlay = 1000 + (uint32_t)(((float)valorGrabado / 1947.0f) * 500.0f);
    else
        pulsoPlay = 1500 + (uint32_t)(((float)(valorGrabado - 2147) / 1948.0f) * 500.0f);

    // 4. Actualizar Servo
    TIM_UpdateMatchValue(LPC_TIM2, 1, pulsoPlay);

    // 5. Actualizar DAC (Para el Osciloscopio)
    // El DAC es de 10 bits, el valorGrabado es de 12 bits.
    // Shift derecha 2 bits para ajustar escala.
    DAC_UpdateValue(LPC_DAC, (valorGrabado >> 2));

    // 6. Avanzar índice
    indicePlay++;
    if (indicePlay >= BUFFER_SIZE)
    {
        indicePlay = 0; 
    }
}

// ---------------------------------------------------------------------------
// EINT1 — Cambio de Modo
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

void EINT1_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT1); 
    for(volatile int i=0; i<100000; i++); // Anti-rebote

    if (modo == 1) 
    {
        // PASAR A GRABAR (Modo 0)
        modo = 0;
        TIM_Cmd(LPC_TIM0, DISABLE);
        ConfDMA(); 
        GPDMA_ChannelCmd(0, ENABLE);
        GPIO_SetValue(0, (1<<LED_PIN)); // LED ON
    }
    else 
    {
        // PASAR A REPRODUCIR (Modo 1)
        modo = 1;
        indicePlay = 0;
        GPDMA_ChannelCmd(0, DISABLE);
        TIM_Cmd(LPC_TIM0, ENABLE);
        GPIO_ClearValue(0, (1<<LED_PIN)); // LED OFF
    }
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(void)
{
    ConfGPIO();
    
    // LED ON al inicio (Modo Grabar)

    ConfDAC();    // Configurar P0.26
    ConfTIMER2(); // PWM Servo
    ConfTIMER0(); // Reproductor
    ConfDMA();    // Buffer
    ConfADC();    // ADC
    ConfEINT1();  // Botón

    while (1)
    {
       
    }
}