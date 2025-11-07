#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_nvic.h"

#define ADC_CH_X 2       // Eje X → AD0.2 (P0.25)
#define SERVO_PIN 9      // Servo → P0.9

#define LED1_PORT 0
#define LED1_PIN 22
#define LED2_PORT 3
#define LED2_PIN 25
#define LED3_PORT 3
#define LED3_PIN 26

#define N_MUESTRAS 20

volatile uint16_t bufferADC[N_MUESTRAS];
volatile uint8_t indiceADC = 0;
volatile uint16_t valorADC = 0;
volatile uint32_t pulso = 1500;

// ================== INTERRUPCIÓN ADC ==================
void ADC_IRQHandler(void) {
    while(!ADC_ChannelGetStatus(LPC_ADC, ADC_CH_X, ADC_DATA_DONE));

    valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH_X);
    bufferADC[indiceADC++] = valorADC;
    if(indiceADC >= N_MUESTRAS) indiceADC = 0;

    uint32_t suma = 0;
    for(int i = 0; i < N_MUESTRAS; i++)
        suma += bufferADC[i];
    uint16_t promedio = suma / N_MUESTRAS;

    // Mapea 0–4095 → 1000–2000 µs
    pulso = 1000 + ((uint32_t)promedio * 1000 / 4095);

    // Actualiza MATCH1 (PWM)
    TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);

    // LEDs según rango del pulso
    if(pulso >= 1000 && pulso < 1365){
        GPIO_SetValue(LED1_PORT, (1<<LED1_PIN));
        GPIO_ClearValue(LED2_PORT, (1<<LED2_PIN));
        GPIO_ClearValue(LED3_PORT, (1<<LED3_PIN));
    } else if(pulso >= 1365 && pulso < 1730){
        GPIO_SetValue(LED2_PORT, (1<<LED2_PIN));
        GPIO_ClearValue(LED1_PORT, (1<<LED1_PIN));
        GPIO_ClearValue(LED3_PORT, (1<<LED3_PIN));
    } else {
        GPIO_SetValue(LED3_PORT, (1<<LED3_PIN));
        GPIO_ClearValue(LED1_PORT, (1<<LED1_PIN));
        GPIO_ClearValue(LED2_PORT, (1<<LED2_PIN));
    }
}

// ================== CONFIGURACIÓN GPIO ==================
void ConfGPIO(void) {
    PINSEL_CFG_Type PinCfg;

    // LEDS
    GPIO_SetDir(LED1_PORT, 1<<LED1_PIN, 1);
    GPIO_SetDir(LED2_PORT, 1<<LED2_PIN, 1);
    GPIO_SetDir(LED3_PORT, 1<<LED3_PIN, 1);

    // Servo
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = SERVO_PIN;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<SERVO_PIN), 1);
}

// ================== CONFIGURACIÓN TIMER2 (PWM SOFTWARE) ==================
void ConfTIMER2(void) {
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;

    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MR0 = periodo 20 ms
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // MR1 = pulso inicial 1500 µs
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

// ================== CONFIGURACIÓN ADC ==================
void ConfADC(void) {
    PINSEL_CFG_Type PinCfg;

    // Eje X (P0.25 -> AD0.2)
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 25;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_ChannelCmd(LPC_ADC, ADC_CH_X, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADGINTEN, ENABLE);

    NVIC_EnableIRQ(ADC_IRQn);
}


void main(){

    while();
}