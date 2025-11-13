/*
Servo 1 | P0.9 | TIMER2 | MR1
Servo 2 | P0.8 | TIMER2 | MR2
Servo 3 | P0.7 | TIMER2 | MR3
Servo 4 | P0.6 | TIMER3 | MR1
 */


#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"

// Definiciones de pines y canales
#define SERVO_PIN1  9   // P0.9  -> Servo 1
#define SERVO_PIN2  8   // P0.8  -> Servo 2
#define SERVO_PIN3  7   // P0.7  -> Servo 3
#define SERVO_PIN4  6   // P0.6  -> Servo 4

#define ADC_CH1     4   // AD0.4 -> P1.30 -> Pot1
#define ADC_CH2     5   // AD0.5 -> P1.31 -> Pot2
#define ADC_CH3     2   // AD0.2 -> P0.25 -> Pot3
#define ADC_CH4     3   // AD0.3 -> P0.26 -> Pot4

#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

// Variables globales
volatile uint16_t valorADC1 = 0, valorADC2 = 0, valorADC3 = 0, valorADC4 = 0;
volatile uint32_t pulso1 = 1500, pulso2 = 1500, pulso3 = 1500, pulso4 = 1500;

// Configuración de GPIO (servos)
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0; // GPIO
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    // Configurar los 4 pines de salida para los servos
    uint8_t pins[] = {SERVO_PIN1, SERVO_PIN2, SERVO_PIN3, SERVO_PIN4};
    for (int i = 0; i < 4; i++) {
        PinCfg.Pinnum = pins[i];
        PINSEL_ConfigPin(&PinCfg);
        GPIO_SetDir(0, (1 << pins[i]), 1);
    }
}

// Configuración ADC (4 potenciómetros)
void ConfADC(void)
{
    PINSEL_CFG_Type PinCfg;

    // AD0.4 -> P1.30 (Pot1)
    PinCfg.Portnum = 1; PinCfg.Pinnum = 30; PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE; PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // AD0.5 -> P1.31 (Pot2)
    PinCfg.Pinnum = 31; PINSEL_ConfigPin(&PinCfg);

    // AD0.2 -> P0.25 (Pot3)
    PinCfg.Portnum = 0; PinCfg.Pinnum = 25; PinCfg.Funcnum = 1;
    PINSEL_ConfigPin(&PinCfg);

    // AD0.3 -> P0.26 (Pot4)
    PinCfg.Pinnum = 26; PINSEL_ConfigPin(&PinCfg);

    // Inicializar ADC
    ADC_Init(LPC_ADC, 1000);
    ADC_BurstCmd(LPC_ADC, ENABLE);

    ADC_ChannelCmd(LPC_ADC, ADC_CH1, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH2, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH3, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH4, ENABLE);

    ADC_IntConfig(LPC_ADC, ADC_CH1, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH2, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH3, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH4, ENABLE);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
    NVIC_EnableIRQ(ADC_IRQn);
}

// Interrupción ADC
void ADC_IRQHandler(void)
{
    // Mismo mapeo 0–4095 → 1000–2000 µs para cada canal

    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH1, 1)) {
        valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH1);
        pulso1 = 1000 + ((uint32_t)valorADC1 * 1000 / 4095);
        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso1);
    }

    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH2, 1)) {
        valorADC2 = ADC_ChannelGetData(LPC_ADC, ADC_CH2);
        pulso2 = 1000 + ((uint32_t)valorADC2 * 1000 / 4095);
        TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);
    }

    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH3, 1)) {
        valorADC3 = ADC_ChannelGetData(LPC_ADC, ADC_CH3);
        pulso3 = 1000 + ((uint32_t)valorADC3 * 1000 / 4095);
        TIM_UpdateMatchValue(LPC_TIM2, 3, pulso3);
    }

    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH4, 1)) {
        valorADC4 = ADC_ChannelGetData(LPC_ADC, ADC_CH4);
        pulso4 = 1000 + ((uint32_t)valorADC4 * 1000 / 4095);
        TIM_UpdateMatchValue(LPC_TIM3, 1, pulso4);
    }
}

// Configuración TIMER2 (3 servos)
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type cfg;
    cfg.PrescaleOption = TIM_PRESCALE_USVAL;
    cfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &cfg);

    // MATCH0 -> 20 ms (periodo)
    TIM_MATCHCFG_Type m0;
    m0.MatchChannel = 0; m0.IntOnMatch = ENABLE; m0.ResetOnMatch = ENABLE;
    m0.StopOnMatch = DISABLE; m0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &m0);

    // MATCH1–3 -> servos
    for (int ch = 1; ch <= 3; ch++) {
        TIM_MATCHCFG_Type m;
        m.MatchChannel = ch; m.IntOnMatch = ENABLE; m.ResetOnMatch = DISABLE;
        m.StopOnMatch = DISABLE; m.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
        m.MatchValue = 1500;
        TIM_ConfigMatch(LPC_TIM2, &m);
    }

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// Configuración TIMER3 (servo4)
void ConfTIMER3(void)
{
    TIM_TIMERCFG_Type cfg;
    cfg.PrescaleOption = TIM_PRESCALE_USVAL;
    cfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &cfg);

    // MATCH0 -> periodo (20 ms)
    TIM_MATCHCFG_Type m0;
    m0.MatchChannel = 0; m0.IntOnMatch = ENABLE; m0.ResetOnMatch = ENABLE;
    m0.StopOnMatch = DISABLE; m0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM3, &m0);

    // MATCH1 -> servo 4
    TIM_MATCHCFG_Type m1;
    m1.MatchChannel = 1; m1.IntOnMatch = ENABLE; m1.ResetOnMatch = DISABLE;
    m1.StopOnMatch = DISABLE; m1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    m1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM3, &m1);

    NVIC_EnableIRQ(TIMER3_IRQn);
    TIM_Cmd(LPC_TIM3, ENABLE);
}

// Interrupción TIMER2 (servos 1–3)
void TIMER2_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1<<SERVO_PIN1)|(1<<SERVO_PIN2)|(1<<SERVO_PIN3));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN1));
    }
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN2));
    }
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR3_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR3_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN3));
    }
}

// Interrupción TIMER3 (servo 4)
void TIMER3_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM3, TIM_MR0_INT)) {
        TIM_ClearIntPending(LPC_TIM3, TIM_MR0_INT);
        GPIO_SetValue(0, (1<<SERVO_PIN4));
    }

    if (TIM_GetIntStatus(LPC_TIM3, TIM_MR1_INT)) {
        TIM_ClearIntPending(LPC_TIM3, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN4));
    }
}

// main()
int main(void)
{
    ConfGPIO();
    ConfADC();
    ConfTIMER2();
    ConfTIMER3();

    while (1) {
    }
}
