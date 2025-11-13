/*
 * Proyecto: Control de 3 servomotores con 3 potenciómetros - LPC1769
 * Autor: [Tu nombre]
 *
 * Descripción:
 * Control de tres servos mediante tres potenciómetros usando ADC en modo burst
 * y generación de PWM por software con el Timer2.
 *
 * Conexiones:
 *  - Potenciómetro 1 → P1.30 (AD0.4)
 *  - Potenciómetro 2 → P1.31 (AD0.5)
 *  - Potenciómetro 3 → P0.25 (AD0.2)
 *  - Servo 1 → P0.9
 *  - Servo 2 → P0.8
 *  - Servo 3 → P0.7
 *  - GND común y VCC 5V (para servos)
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"

// ---------------------------------------------------------------------------
// Definiciones
// ---------------------------------------------------------------------------
#define SERVO_PIN   9   // P0.9  -> Servo 1
#define SERVO_PIN2  8   // P0.8  -> Servo 2
#define SERVO_PIN3  7   // P0.7  -> Servo 3

#define ADC_CH      4   // AD0.4 -> P1.30 -> Potenciómetro 1
#define ADC_CH5     5   // AD0.5 -> P1.31 -> Potenciómetro 2
#define ADC_CH2     2   // AD0.2 -> P0.25 -> Potenciómetro 3

#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

// ---------------------------------------------------------------------------
// Variables globales
// ---------------------------------------------------------------------------
volatile uint16_t valorADC1 = 0;
volatile uint16_t valorADC2 = 0;
volatile uint16_t valorADC3 = 0;

volatile uint32_t pulso = 1500;
volatile uint32_t pulso2 = 1500;
volatile uint32_t pulso3 = 1500;

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

    // Servo 2 (P0.8)
    PinCfg.Pinnum = SERVO_PIN2;
    PINSEL_ConfigPin(&PinCfg);

    // Servo 3 (P0.7)
    PinCfg.Pinnum = SERVO_PIN3;
    PINSEL_ConfigPin(&PinCfg);

    // Configurar pines como salida
    GPIO_SetDir(0, (1<<SERVO_PIN) | (1<<SERVO_PIN2) | (1<<SERVO_PIN3), 1);
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

    // Potenciómetro 2 -> P1.31 (AD0.5)
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);

    // Potenciómetro 3 -> P0.25 (AD0.2)
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 25;
    PinCfg.Funcnum = 1; // Función AD0.2
    PINSEL_ConfigPin(&PinCfg);

    // Inicializar ADC
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

// ---------------------------------------------------------------------------
// Interrupción ADC
// ---------------------------------------------------------------------------
void ADC_IRQHandler(void)
{
    // --- Canal AD0.4 (P1.30) ---
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);
        if (valorADC1 >= 1948 && valorADC1 < 2147)
            pulso = 1500;
        else if (valorADC1 < 1948)
            pulso = 1000 + ((uint32_t)valorADC1 * 500 / 1947);
        else
            pulso = 1500 + ((uint32_t)valorADC1 * 500 / 1947);

        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }

    // --- Canal AD0.5 (P1.31) ---
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH5, 1))
    {
        valorADC2 = ADC_ChannelGetData(LPC_ADC, ADC_CH5);
        if (valorADC2 >= 1948 && valorADC2 < 2147)
            pulso2 = 1500;
        else if (valorADC2 < 1948)
            pulso2 = 1000 + ((uint32_t)valorADC2 * 500 / 1947);
        else
            pulso2 = 1500 + ((uint32_t)valorADC2 * 500 / 1947);

        TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);
    }

    // --- Canal AD0.2 (P0.25) ---
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH2, 1))
    {
        valorADC3 = ADC_ChannelGetData(LPC_ADC, ADC_CH2);
        if (valorADC3 >= 1948 && valorADC3 < 2147)
            pulso3 = 1500;
        else if (valorADC3 < 1948)
            pulso3 = 1000 + ((uint32_t)valorADC3 * 500 / 1947);
        else
            pulso3 = 1500 + ((uint32_t)valorADC3 * 500 / 1947);

        TIM_UpdateMatchValue(LPC_TIM2, 3, pulso3);
    }
}

// ---------------------------------------------------------------------------
// Configuración Timer2 (PWM software)
// ---------------------------------------------------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // --- MATCH0: periodo 20 ms ---
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // --- MATCH1: Servo 1 ---
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    // --- MATCH2: Servo 2 ---
    TIM_MATCHCFG_Type match2;
    match2.MatchChannel = 2;
    match2.IntOnMatch = ENABLE;
    match2.ResetOnMatch = DISABLE;
    match2.StopOnMatch = DISABLE;
    match2.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match2.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match2);

    // --- MATCH3: Servo 3 ---
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

// ---------------------------------------------------------------------------
// Interrupción Timer2
// ---------------------------------------------------------------------------
void TIMER2_IRQHandler(void)
{
    // MATCH0 → reinicia ciclo PWM
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1<<SERVO_PIN));
        GPIO_SetValue(0, (1<<SERVO_PIN2));
        GPIO_SetValue(0, (1<<SERVO_PIN3));
    }

    // MATCH1 → fin pulso servo 1
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN));
    }

    // MATCH2 → fin pulso servo 2
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN2));
    }

    // MATCH3 → fin pulso servo 3
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR3_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR3_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN3));
    }
}

// ---------------------------------------------------------------------------
// Función principal
// ---------------------------------------------------------------------------
int main(void)
{
    ConfGPIO();    // Configurar pines de servos
    ConfADC();     // Configurar ADC (3 potenciómetros)
    ConfTIMER2();  // Configurar Timer2 para PWM software

    while (1)
    {
        // Loop vacío, control manejado por interrupciones
    }
}
