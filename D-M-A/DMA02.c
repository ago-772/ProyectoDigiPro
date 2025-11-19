/*
 * Proyecto: Servo + ADC + DMA + EINT1
 * MCU: LPC1769
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
// DMA (ADC → RAM)
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
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_BurstCmd(LPC_ADC, ENABLE);

    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);

    NVIC_EnableIRQ(ADC_IRQn);
}

// ---------------------------------------------------------------------------
// IRQ ADC — fórmula EXACTA pedida por vos
// ---------------------------------------------------------------------------
void ADC_IRQHandler(void)
{
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        if (valorADC >= 1948 && valorADC < 2147)
            pulso = 1500;
        else if (valorADC < 1948)
            pulso = 1000 + (uint32_t)(((float)valorADC / 1947.0f) * 500.0f);
        else
            pulso = 1500 + (uint32_t)(((float)(valorADC - 2147) / 1948.0f) * 500.0f);

        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }
}

// ---------------------------------------------------------------------------
// TIMER2 — PWM software para servo
// ---------------------------------------------------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MATCH0 → 20ms
    TIM_MATCHCFG_Type m0;
    m0.MatchChannel = 0;
    m0.IntOnMatch   = ENABLE;
    m0.ResetOnMatch = ENABLE;
    m0.StopOnMatch  = DISABLE;
    m0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m0.MatchValue   = 20000;
    TIM_ConfigMatch(LPC_TIM2, &m0);

    // MATCH1 → servo
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
// EINT1 — configuración
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
// Handler EINT1 — EXACTO como lo pediste
// ---------------------------------------------------------------------------


void EINT1_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT1);   // limpiamos interrupción (drivers)

    if (modo == 0)
    {
        // ----- ENTRAR A GRABAR -----
        modo = 1;
        indicePlay = 0;

        // Pausar reproducción
        TIM_Cmd(LPC_TIM0, DISABLE);

        // Preparar DMA para volver a llenar el buffer
        ConfDMA();
        GPDMA_ChannelCmd(0, ENABLE);
        GPIO_SetValue(0,1<<22);

    }
    else if (modo == 1)
    {
        // ----- ENTRAR A REPRODUCIR -----
        modo = 2;
        indicePlay = 0;

        // Detener DMA (ya no grabamos)
        GPDMA_ChannelCmd(0, DISABLE);

        // Iniciar reproducción automática
        TIM_Cmd(LPC_TIM0, ENABLE);
        GPIO_ClearValue(0,1<<22);

    }
}

// ---------------------------------------------------------------------------
// TIMER0
// ---------------------------------------------------------------------------

void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;   // 1us

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    // MATCH0 → interrupción cada 20ms
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.MatchValue   = 20000;  // 20 ms
    match0.IntOnMatch   = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch  = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

    TIM_ConfigMatch(LPC_TIM0, &match0);

    NVIC_EnableIRQ(TIMER0_IRQn);
}

// ---------------------------------------------------------------------------
// TIMER0
// ---------------------------------------------------------------------------

void TIMER0_IRQHandler(void)
{
    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

    // Leer valor grabado
    uint32_t valorGrabado = adcBuffer[indicePlay];

    uint32_t pulsoPlay;

    // ---- MISMA FÓRMULA EXACTA QUE EN EL ADC ----
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
        indicePlay = 0;
}


// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(void)
{
	GPIO_SetDir(0,1<<22,1);

    ConfGPIO();
    ConfTIMER2();
    ConfDMA();     // solo si se usa modo grabar
    ConfADC();
    ConfEINT1();
    ConfTIMER0();


    while (1)
    {
        // todo manejado por IRQ
    }
}
