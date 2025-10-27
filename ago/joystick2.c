#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"

#define SERVO_MIN_US 1000      // 1.0 ms
#define SERVO_MAX_US 2000      // 2.0 ms
#define PWM_PERIOD_US 20000    // 20 ms = 50 Hz

void configADC(void);
void configPWM(void);
void configMatch1(void);

int main(void) {
    SystemInit();

    configADC();
    configPWM();
    configMatch1();

    TIM_Cmd(LPC_TIM2, ENABLE);   // Inicia el Timer2

    while (1) {
        // nada, todo se hace por hardware e interrupciones
    }
}

/*---------------------------------------------
  CONFIGURACIÓN DEL ADC (canal 0, P0.23)
---------------------------------------------*/
void configADC(void) {
    PINSEL_CFG_Type pinCfg;
    pinCfg.Funcnum   = 1;       // Función ADC
    pinCfg.OpenDrain = 0;
    pinCfg.Pinmode   = 0;
    pinCfg.Portnum   = 0;
    pinCfg.Pinnum    = 23;
    PINSEL_ConfigPin(&pinCfg);

    ADC_Init(LPC_ADC, 200000);  // 200 kHz
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, DISABLE);
}

/*---------------------------------------------
  CONFIGURACIÓN PWM con TIMER2 (MAT2.0)
---------------------------------------------*/
void configPWM(void) {
    PINSEL_CFG_Type pinCfg;
    pinCfg.Funcnum   = 3;      // P0.6 → MAT2.0
    pinCfg.OpenDrain = 0;
    pinCfg.Pinmode   = 0;
    pinCfg.Portnum   = 0;
    pinCfg.Pinnum    = 6;
    PINSEL_ConfigPin(&pinCfg);

    TIM_TIMERCFG_Type timerCfg;
    TIM_MATCHCFG_Type matchCfg;

    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue  = 1;           // 1 tick = 1 µs
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MATCH0 → período de 20 ms (reset del timer)
    matchCfg.MatchChannel = 0;
    matchCfg.IntOnMatch   = DISABLE;
    matchCfg.StopOnMatch  = DISABLE;
    matchCfg.ResetOnMatch = ENABLE;
    matchCfg.ExtMatchOutputType = TIM_EXTMATCH_SET; // sube pin
    matchCfg.MatchValue   = PWM_PERIOD_US;
    TIM_ConfigMatch(LPC_TIM2, &matchCfg);

    // MATCH1 → conversión ADC cada 10 ms
    matchCfg.MatchChannel = 1;
    matchCfg.IntOnMatch   = ENABLE;
    matchCfg.StopOnMatch  = DISABLE;
    matchCfg.ResetOnMatch = DISABLE;
    matchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    matchCfg.MatchValue   = 10000;
    TIM_ConfigMatch(LPC_TIM2, &matchCfg);

    // MATCH2 → controla el ancho del pulso
    matchCfg.MatchChannel = 2;
    matchCfg.IntOnMatch   = DISABLE;
    matchCfg.StopOnMatch  = DISABLE;
    matchCfg.ResetOnMatch = DISABLE;
    matchCfg.ExtMatchOutputType = TIM_EXTMATCH_CLEAR; // baja pin
    matchCfg.MatchValue   = 1500;                     // 1.5 ms (centro)
    TIM_ConfigMatch(LPC_TIM2, &matchCfg);

    NVIC_EnableIRQ(TIMER2_IRQn);
}

/*---------------------------------------------
  INTERUPCIÓN TIMER2: inicia conversión ADC
---------------------------------------------*/
void TIMER2_IRQHandler(void) {
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        ADC_StartCmd(LPC_ADC, ADC_START_NOW);
        while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)));
        uint16_t valor = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);

        // Escala: 0–4095 → 1000–2000 µs
        uint32_t pulso = SERVO_MIN_US + ((valor * (SERVO_MAX_US - SERVO_MIN_US)) / 4095);
        TIM_UpdateMatchValue(LPC_TIM2, 2, pulso);
    }
}
