/*
Convertir el voltaje del potenciómetro (0–3,3 V) a un valor digital (0–4095).
Mapear ese valor a un pulso de 1 ms a 2 ms (rango típico de servo).
Generar la señal PWM con el TIMER2 sobre MAT2.0 (P0.6).
Actualizar el ancho de pulso cada cierto tiempo, automáticamente por interrupciones.
*/

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"


// Definiciones y constantes
#define ADC_CH      ADC_CHANNEL_0    // Canal 0 -> P0.23
#define SERVO_PIN   6                // P0.6 -> MAT2.0
#define SERVO_PORT  0

#define SERVO_MIN_US 1000            // 1.0 ms  -> ángulo mínimo
#define SERVO_MAX_US 2000            // 2.0 ms  -> ángulo máximo
#define PWM_PERIOD_US 20000          // 20 ms (50 Hz)

volatile uint16_t valorADC = 0;

void confPINSEL(void);
void confADC(void);
void confTIMER2(void);
void TIMER2_IRQHandler(void);
void ADC_IRQHandler(void);

int main(void) {
    SystemInit();        // Inicializa reloj del sistema
    confPINSEL();        // Configura funciones de pines
    confADC();           // Inicializa ADC
    confTIMER2();        // Configura timer y PWM

    while(1) {
        __WFI();         // Espera interrupciones (bajo consumo)
    }
}

void confPINSEL(void) {
    PINSEL_CFG_Type PinCfg;

    // --- Pin P0.23 como AD0.0 (entrada analógica) ---
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 23;
    PinCfg.Funcnum = 1;          // Función 01 -> AD0.0
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PINSEL_ConfigPin(&PinCfg);

    // --- Pin P0.6 como MAT2.0 (salida PWM por timer) ---
    PinCfg.Portnum = SERVO_PORT;
    PinCfg.Pinnum = SERVO_PIN;
    PinCfg.Funcnum = 3;          // Función 11 -> MAT2.0
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PINSEL_ConfigPin(&PinCfg);
}

// Configuración del ADC
void confADC(void) {
    ADC_Init(LPC_ADC, 200000);                      // Frecuencia ADC: 200kHz
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);        // Habilita canal 0
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);   // Interrupción canal 0
    NVIC_EnableIRQ(ADC_IRQn);                       // Habilita interrupción global del ADC
}

// Configuración del TIMER2
void confTIMER2(void) {
    TIM_TIMERCFG_Type TimerCfg;
    TIM_MATCHCFG_Type MatchCfg;

    // --- Modo TIMER: cuenta en microsegundos ---
    TimerCfg.PrescaleOption = TIM_PRESCALE_USVAL; // 1 tick = 1 µs
    TimerCfg.PrescaleValue  = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TimerCfg);

    // --- MATCH0: define el período del PWM (20 ms) ---
    MatchCfg.MatchChannel = 0;
    MatchCfg.IntOnMatch = DISABLE;
    MatchCfg.StopOnMatch = DISABLE;
    MatchCfg.ResetOnMatch = ENABLE;             // reinicia el timer al llegar a MR0
    MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_SET;
    MatchCfg.MatchValue = PWM_PERIOD_US;        // 20 000 µs
    TIM_ConfigMatch(LPC_TIM2, &MatchCfg);

    // --- MATCH1: disparo periódico de conversión ADC (cada 10 ms) ---
    MatchCfg.MatchChannel = 1;
    MatchCfg.IntOnMatch = ENABLE;               // genera interrupción
    MatchCfg.StopOnMatch = DISABLE;
    MatchCfg.ResetOnMatch = DISABLE;
    MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    MatchCfg.MatchValue = 10000;                // cada 10 ms
    TIM_ConfigMatch(LPC_TIM2, &MatchCfg);

    // --- MATCH2: controla el ancho del pulso PWM ---
    MatchCfg.MatchChannel = 2;
    MatchCfg.IntOnMatch = DISABLE;
    MatchCfg.StopOnMatch = DISABLE;
    MatchCfg.ResetOnMatch = DISABLE;
    MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_CLEAR; // alterna salida MAT2.0
    MatchCfg.MatchValue = 1500;                      // inicio centrado (1.5 ms)
    TIM_ConfigMatch(LPC_TIM2, &MatchCfg);

    // Habilita interrupción del Timer2
    NVIC_EnableIRQ(TIMER2_IRQn);

    // Inicia timer
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// Interrupción del TIMER2
void TIMER2_IRQHandler(void) {
    // --- Si coincide con MATCH1 (cada 10ms) ---
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);

        // Inicia una conversión ADC
        ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    }
}

// Interrupción del ADC
void ADC_IRQHandler(void) {
    // Lee valor del canal 0
    valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH);

    // Mapea 0–4095 → 1000–2000 µs
    uint32_t pulso = SERVO_MIN_US + ((valorADC * (SERVO_MAX_US - SERVO_MIN_US)) / 4095);

    // Actualiza el valor del MATCH2 (ancho de pulso)
    TIM_UpdateMatchValue(LPC_TIM2, 2, pulso);
}


//MATCH0 = 20 000 µs → define el período total del PWM (20 ms).
