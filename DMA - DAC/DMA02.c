/*
 * Proyecto: Grabadora de Servo 3 Estados (Manual, Rec, Play) - LPC1769
 *
 * Estados (Controlados por EINT1 - Botón):
 * 0 - MANUAL: Potenciómetro -> Servo. (Led P0.22 Apagado)
 * 1 - GRABANDO: Potenciómetro -> Servo + Guardado en RAM. (Led P0.22 Encendido)
 * 2 - REPRODUCIENDO: RAM -> DMA -> Servo. (Led P0.22 Apagado)
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_exti.h"

// ---------------------------------------------------------------------------
// Definiciones
#define SERVO_PIN   9   // P0.9
#define LED_DEBUG   22  // P0.22
#define ADC_CH      4   // AD0.4 -> P1.30

#define PWM_PERIOD_US   20000   // 20ms
#define RECORD_SEC      10      // 10 Segundos
#define BUFFER_SIZE     500

// ---------------------------------------------------------------------------
// Variables Globales
volatile uint16_t valorADC1 = 0;
volatile uint32_t pulso_actual = 1500;

// Buffer para grabar movimientos
uint32_t servo_buffer[BUFFER_SIZE];
volatile uint32_t recordIndex = 0; // Indice de grabación

// Variable de Estado (0: Manual, 1: Rec, 2: Play)
volatile uint8_t systemMode = 0;

// Estructuras DMA
GPDMA_Channel_CFG_Type GPDMACfg;
GPDMA_LLI_Type DMA_LLI_Struct;

// ---------------------------------------------------------------------------
// Configuración GPIO
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0; // GPIO
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    // Servo P0.9
    PinCfg.Pinnum = SERVO_PIN;
    PINSEL_ConfigPin(&PinCfg);

    // LED Debug P0.22
    PinCfg.Pinnum = LED_DEBUG;
    PINSEL_ConfigPin(&PinCfg);

    // Dirección Salida
    GPIO_SetDir(0, (1<<SERVO_PIN) | (1<<LED_DEBUG), 1);
}

// ---------------------------------------------------------------------------
// Configuración ADC
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
    ADC_BurstCmd(LPC_ADC, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
    NVIC_EnableIRQ(ADC_IRQn);
}

// ---------------------------------------------------------------------------
// Configuración Timer2 (PWM Generation)
void ConfTIMER2_PWM(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MATCH0: Periodo (20 ms)
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = PWM_PERIOD_US;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // MATCH1: Ancho de Pulso (Servo)
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// ---------------------------------------------------------------------------
// Configuración Timer0 (Tick de Grabación / Trigger de Reproducción)
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    // La configuración de interrupciones se maneja dinámicamente en EINT1
    match0.IntOnMatch = DISABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = PWM_PERIOD_US; // 20ms
    TIM_ConfigMatch(LPC_TIM0, &match0);

    NVIC_EnableIRQ(TIMER0_IRQn);
    // TIM_Cmd se activa en EINT1
}

// ---------------------------------------------------------------------------
// Configuración DMA
void ConfDMA(void)
{
    GPDMA_Init();

    // Lista Enlazada Circular
    DMA_LLI_Struct.SrcAddr = (uint32_t)servo_buffer;
    DMA_LLI_Struct.DstAddr = (uint32_t)&(LPC_TIM2->MR1); // Destino: Timer2 Match1
    DMA_LLI_Struct.NextLLI = (uint32_t)&DMA_LLI_Struct;  // Circular
    DMA_LLI_Struct.Control = BUFFER_SIZE
                           | (2 << 18) // Source Width 32
                           | (2 << 21) // Dest Width 32
                           | (1 << 26) // Source Increment
                           | (0 << 27); // Dest Inc Disabled

    GPDMACfg.ChannelNum = 0;
    GPDMACfg.SrcMemAddr = (uint32_t)servo_buffer;
    GPDMACfg.DstMemAddr = (uint32_t)&(LPC_TIM2->MR1);
    GPDMACfg.TransferSize = BUFFER_SIZE;
    GPDMACfg.TransferWidth = 0;
    GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
    GPDMACfg.SrcConn = 0;
    GPDMACfg.DstConn = GPDMA_CONN_MAT0_0; // Trigger: Timer0 Match 0
    GPDMACfg.DMALLI = (uint32_t)&DMA_LLI_Struct;

    GPDMA_Setup(&GPDMACfg);
}

// ---------------------------------------------------------------------------
// Configuración EINT1
void ConfEINT1(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 11;
    PinCfg.Funcnum = 1; // EINT1
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    EXTI_Init();
    EXTI_InitTypeDef ExtiCfg;
    ExtiCfg.EXTI_Line = EXTI_EINT1;
    ExtiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    ExtiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&ExtiCfg);

    NVIC_EnableIRQ(EINT1_IRQn);
}

// ---------------------------------------------------------------------------
// IRQ EINT1: Máquina de Estados
void EINT1_IRQHandler(void)
{
    // Simple retardo anti-rebote
    for (volatile int i = 0; i < 1000000; i++);

    EXTI_ClearEXTIFlag(EXTI_EINT1);

    systemMode = (systemMode + 1) % 3; // Cicla 0 -> 1 -> 2 -> 0

    if (systemMode == 0)        // MODO MANUAL
    {
        GPIO_ClearValue(0, 1<<LED_DEBUG);

        GPDMA_ChannelCmd(0, DISABLE);           // Detiene DMA
        TIM_Cmd(LPC_TIM0, DISABLE);             // Detiene Timer0 (grabación)

        //LPC_TIM0->MCR = 0;                      // Limpia config de Timer0 (evita interrupciones o requests fantasma)

        NVIC_EnableIRQ(ADC_IRQn);               // Habilita control por ADC
    }
    else if (systemMode == 1)   // MODO GRABANDO
    {
        GPIO_SetValue(0, 1<<LED_DEBUG);         // LED ON

        GPDMA_ChannelCmd(0, DISABLE);           // Detiene DMA
        recordIndex = 0;                        // Reinicia índice
        NVIC_EnableIRQ(ADC_IRQn);               // ADC activo

        // Configurar Timer0 para interrumpir y grabar
        TIM_ResetCounter(LPC_TIM0);

        //LPC_TIM0->MCR = 3; // Bit 0 (Int) + Bit 1 (Reset)

        TIM_Cmd(LPC_TIM0, ENABLE);    // Inicia Timer0
    }
    else // systemMode == 2: MODO REPRODUCIENDO
    {
        GPIO_ClearValue(0, 1<<LED_DEBUG);

        NVIC_DisableIRQ(ADC_IRQn);              // Deshabilita ADC

        GPDMA_Setup(&GPDMACfg);                 // Reinicia el DMA desde el principio
        GPDMA_ChannelCmd(0, ENABLE);            // Inicia DMA

        // Configurar Timer0 para disparar el DMA (Sin interrupción CPU)
        TIM_ResetCounter(LPC_TIM0);

        //LPC_TIM0->MCR = (1 << 1) | (1 << 3); // Bit 1 (Reset) + Bit 3 (DMA Req)

        TIM_Cmd(LPC_TIM0, ENABLE);    // El Timer0 DEBE correr para sincronizar
    }
}


// ---------------------------------------------------------------------------
// IRQ ADC
void ADC_IRQHandler(void)
{
    // Solo actualiza 'pulso' si el DMA no está activo (Modo Manual o Grabando)
    if (systemMode != 2)
    {
        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
        {
            valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);

            uint32_t pulso;

            if (valorADC1 >= 1948 && valorADC1 < 2147)
            {
                pulso = 1500;
            }
            else if (valorADC1 < 1948)
            {

                pulso = 1000 + (uint32_t)(((float)valorADC1 / 1947.0f) * 500.0f);
            }
            else
            {
                pulso = 1500 + (uint32_t)(((float)(valorADC1 - 2147) / 1948.0f) * 500.0f);
            }

            TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
            pulso_actual = pulso;
        }

    }
    else
    {
        // Si estamos en modo DMA, solo limpia el flag leyendo el registro
        if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
            ADC_ChannelGetData(LPC_ADC, ADC_CH);
    }
}

// ---------------------------------------------------------------------------
// IRQ Timer0: Grabación (Solo activa en Modo 1)
void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

        // Solo grabamos si estamos en modo grabación
        if (systemMode == 1)
        {
            servo_buffer[recordIndex] = pulso_actual;
            recordIndex++;
            if (recordIndex >= BUFFER_SIZE) {
                recordIndex = 0; // Buffer circular
            }
        }
    }
}

// ---------------------------------------------------------------------------
// IRQ Timer2: PWM
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
// Main
int main(void)
{
    // Inicializar buffer en centro
    //for(int i=0; i<BUFFER_SIZE; i++) servo_buffer[i] = 1500;

    ConfGPIO();
    ConfDMA();
    ConfADC();
    ConfTIMER2_PWM();
    ConfTIMER0();
    ConfEINT1();

    while (1)
    {
      
    }
}