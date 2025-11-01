#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"

/*
 * 3 servos (base, codo, mano) controlados por 3 potenciómetros
 * mediante el ADC en modo Burst.
 * Se genera PWM por software con Timer2.
 */

// ==== Definición de pines ====
#define SERVO_BASE_PIN  9   // P0.9
#define SERVO_CODO_PIN  8   // P0.8
#define SERVO_MANO_PIN  7   // P0.7

// ==== Canales ADC usados ====
#define ADC_BASE_CH  4      // P1.30
#define ADC_CODO_CH  5      // P1.31
#define ADC_MANO_CH  2      // P0.25

// INTERRUPCIÓN DEL ADC
// ==== Rango de pulsos válidos para el servo ====
#define SERVO_MIN_US 1000    // 1.0 ms  -> ángulo mínimo
#define SERVO_MAX_US 2000    // 2.0 ms  -> ángulo máximo

void ADC_IRQHandler(void)
{
    uint16_t valorADC;       // Variable temporal para lecturas del ADC
    uint32_t pulso;          // Valor en microsegundos para el duty PWM

    // ---- Canal BASE (AD0.4) ----
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_BASE_CH, ADC_DATA_DONE)) {

        // Lee valor ADC (0–4095)
        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_BASE_CH);

        // Mapea 0–4095 → 1000–2000 µs
        pulso = SERVO_MIN_US + ((valorADC * (SERVO_MAX_US - SERVO_MIN_US)) / 4095);

        // Actualiza ancho del pulso (MR1)
        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }

    // ---- Canal CODO (AD0.5) ----
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CODO_CH, ADC_DATA_DONE)) {

        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CODO_CH);
        pulso = SERVO_MIN_US + ((valorADC * (SERVO_MAX_US - SERVO_MIN_US)) / 4095);
        TIM_UpdateMatchValue(LPC_TIM2, 2, pulso);
    }

    // ---- Canal MANO (AD0.2) ----
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_MANO_CH, ADC_DATA_DONE)) {

        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_MANO_CH);
        pulso = SERVO_MIN_US + ((valorADC * (SERVO_MAX_US - SERVO_MIN_US)) / 4095);
        TIM_UpdateMatchValue(LPC_TIM2, 3, pulso);
    }
}


// INTERRUPCIÓN DEL TIMER2
void TIMER2_IRQHandler(void)
{
    // Match 0 → reinicia PWM cada 20 ms
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1<<SERVO_BASE_PIN)|(1<<SERVO_CODO_PIN)|(1<<SERVO_MANO_PIN));
    }

    // Match 1 → fin de pulso servo base
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_BASE_PIN));
    }

    // Match 2 → fin de pulso servo codo
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
        GPIO_ClearValue(0, (1<<SERVO_CODO_PIN));
    }

    // Match 3 → fin de pulso servo mano
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR3_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR3_INT);
        GPIO_ClearValue(0, (1<<SERVO_MANO_PIN));
    }
}

// CONFIGURACIÓN DE GPIO
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;

    // Pines de salida para servos (P0.7, P0.8, P0.9)
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    for (int pin = 7; pin <= 9; pin++) {
        PinCfg.Pinnum = pin;
        PINSEL_ConfigPin(&PinCfg);
    }
    GPIO_SetDir(0, (1<<7)|(1<<8)|(1<<9), 1);
}

// CONFIGURACIÓN DEL TIMER2
void ConfTIMER2(void)
{
    // Estructura base del timer
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;  // 1 µs

    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // ---- MR0: periodo 20 ms ----
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // ---- MR1, MR2, MR3: duty cycles ----
    TIM_MATCHCFG_Type match;
    match.IntOnMatch = ENABLE;
    match.ResetOnMatch = DISABLE;
    match.StopOnMatch = DISABLE;
    match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

    match.MatchChannel = 1; match.MatchValue = 1500; TIM_ConfigMatch(LPC_TIM2, &match);
    match.MatchChannel = 2; match.MatchValue = 1500; TIM_ConfigMatch(LPC_TIM2, &match);
    match.MatchChannel = 3; match.MatchValue = 1500; TIM_ConfigMatch(LPC_TIM2, &match);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// CONFIGURACIÓN DEL ADC (Burst Mode)
void ConfADC(void)
{
    // Pines analógicos: AD0.4 (P1.30), AD0.5 (P1.31), AD0.2 (P0.25)
    PINSEL_CFG_Type PinCfg;

    // AD0.4
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&PinCfg);

    // AD0.5
    PinCfg.Pinnum = 31;
    PinCfg.Funcnum = 3;
    PINSEL_ConfigPin(&PinCfg);

    // AD0.2
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 25;
    PinCfg.Funcnum = 1;
    PINSEL_ConfigPin(&PinCfg);

    // Inicializa ADC
    ADC_Init(LPC_ADC, 200000);                   // 200 kHz
    ADC_BurstCmd(LPC_ADC, ENABLE);               // Burst mode
    ADC_ChannelCmd(LPC_ADC, ADC_BASE_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CODO_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_MANO_CH, ENABLE);

    ADC_IntConfig(LPC_ADC, ADC_ADGINTEN, DISABLE);
    ADC_IntConfig(LPC_ADC, ADC_BASE_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CODO_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_MANO_CH, ENABLE);

    NVIC_EnableIRQ(ADC_IRQn);
}

// MAIN
int main(void)
{
    ConfGPIO();
    ConfTIMER2();
    ConfADC();

    while (1);
}
