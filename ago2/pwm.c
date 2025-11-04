#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"

#define SERVO_MIN 1000   // 1 ms
#define SERVO_MAX 2000   // 2 ms
#define SERVO_PERIOD 20000 // 20 ms = 50 Hz

int main(void) {
    // ==== Configuración del pin para PWM (MAT0.0 en P1.28) ====
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 3;     // Función alternativa MAT0.0
    PinCfg.Portnum = 1;
    PinCfg.Pinnum  = 28;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PINSEL_ConfigPin(&PinCfg);

    // ==== Configuración del ADC (canal 0 en P0.23) ====
    PinCfg.Funcnum = 1;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum  = 23;
    PINSEL_ConfigPin(&PinCfg);
    ADC_Init(LPC_ADC, 200000);          // 200 kHz
    ADC_BurstCmd(LPC_ADC, DISABLE);
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE);

    // ==== Configuración del TIMER ====
    TIM_TIMERCFG_Type TimerCfg;
    TIM_MATCHCFG_Type MatchCfg;

    TimerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    TimerCfg.PrescaleValue  = 1; // Incremento de 1 µs
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TimerCfg);

    MatchCfg.MatchChannel = 0;
    MatchCfg.IntOnMatch = FALSE;
    MatchCfg.ResetOnMatch = TRUE;
    MatchCfg.StopOnMatch = FALSE;
    MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
    MatchCfg.MatchValue = SERVO_MIN;
    TIM_ConfigMatch(LPC_TIM0, &MatchCfg);

    TIM_Cmd(LPC_TIM0, ENABLE);

    // ==== Bucle principal ====
    while (1) {
        // Leer ADC
        ADC_StartCmd(LPC_ADC, ADC_START_NOW);
        while (!(ADC_ChannelGetStatus(LPC_ADC, 0, ADC_DATA_DONE)));
        uint16_t adcValue = ADC_ChannelGetData(LPC_ADC, 0);

        // Mapear 0–4095 a 1000–2000 µs
        uint32_t pulseWidth = SERVO_MIN + ((adcValue * (SERVO_MAX - SERVO_MIN)) / 4095);

        // Actualizar el ancho del pulso
        TIM_UpdateMatchValue(LPC_TIM0, 0, pulseWidth);

        // Esperar un poco
        for (volatile int i = 0; i < 10000; i++);
    }
}
