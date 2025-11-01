#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"

/*
 * potenciometro por el pin - P0.9
 * mediante el ADC (canal AD0.4) y PWM por software con Timer2. - P1.30
 */

#define SERVO_PIN 9        // P0.9
#define ADC_CH    4        // AD0.4 (P1.30)

#define SERVO_MIN_US 1000            // 1.0 ms  -> ángulo mínimo
#define SERVO_MAX_US 2000            // 2.0 ms  -> ángulo máximo

volatile uint16_t valorADC = 0;

// INTERRUPCIÓN ADC
void ADC_IRQHandler(void) {
    // Lee valor del canal 0
    valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH);

    // Mapea 0–4095 → 1000–2000 µs
    uint32_t pulso = SERVO_MIN_US + ((valorADC * (SERVO_MAX_US - SERVO_MIN_US)) / 4095);

    // Actualiza el valor del MATCH2 (ancho de pulso)
    TIM_UpdateMatchValue(LPC_TIM2, 2, pulso);
}

// INTERRUPCIÓN TIMER2
void TIMER2_IRQHandler(void)
{
	//interrupion por MATCH0
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);		//limpiar bandera
        GPIO_SetValue(0, (1<<SERVO_PIN));				//poner en ALTO el pin P0.9
    }

    //interrupion por MATCH1
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);		//limpiar bandera
        GPIO_ClearValue(0, (1<<SERVO_PIN));				//poner en BAJO el pin P0.9
    }
}

// CONFIGURACIÓN GPIO
void ConfGPIO(void)
{
	//configuracion pin P0.9
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinnum = SERVO_PIN;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;


    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<SERVO_PIN), 1);					//configurar como salida
}

// CONFIGURACIÓN TIMER2
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;

    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MR0 = periodo (20 ms)
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;							//en la interrupcion pone el pin en ALTO
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // MR1 = duty inicial
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;							//en la interrupcion pone el pin en BAJO
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// CONFIGURACIÓN ADC
void ConfADC(void)
{
    // Pin P1.30 -> AD0.4
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);       // 200 kHz
    ADC_BurstCmd(LPC_ADC, ENABLE);   // Burst Mode
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);

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
