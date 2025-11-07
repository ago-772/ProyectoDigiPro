#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_nvic.h"

#define ADC_CH_X 2       // Eje X → AD0.2 (P0.25)
#define ADC_CH_Y 3       // Eje Y → AD0.3 (P0.26)
#define SERVO_X_PIN 9    // Servo X → P0.9
#define SERVO_Y_PIN 8    // Servo Y → P0.8

#define LED1_PORT 0
#define LED1_PIN 22
#define LED2_PORT 3
#define LED2_PIN 25
#define LED3_PORT 3
#define LED3_PIN 26

#define N_MUESTRAS 20

volatile uint16_t bufferX[N_MUESTRAS];
volatile uint16_t bufferY[N_MUESTRAS];
volatile uint8_t indice = 0;
volatile uint16_t valorX = 0;
volatile uint16_t valorY = 0;
volatile uint32_t pulsoX = 1500;
volatile uint32_t pulsoY = 1500;

// ================== INTERRUPCIÓN ADC ==================
void ADC_IRQHandler(void) {
    // --- EJE X ---
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while(!ADC_ChannelGetStatus(LPC_ADC, ADC_CH_X, ADC_DATA_DONE));
    valorX = ADC_ChannelGetData(LPC_ADC, ADC_CH_X);

    // --- EJE Y ---
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while(!ADC_ChannelGetStatus(LPC_ADC, ADC_CH_Y, ADC_DATA_DONE));
    valorY = ADC_ChannelGetData(LPC_ADC, ADC_CH_Y);

    // Guarda en buffers
    bufferX[indice] = valorX;
    bufferY[indice] = valorY;
    indice++;
if (indice >= N_MUESTRAS) {
    indice = 0;

    uint32_t sumaX = 0, sumaY = 0;
    for(int i = 0; i < N_MUESTRAS; i++){
        sumaX += bufferX[i];
        sumaY += bufferY[i];
    }

    uint16_t promedioX = sumaX / N_MUESTRAS;
    uint16_t promedioY = sumaY / N_MUESTRAS;

    // Mapea 0–4095 → 1000–2000 µs
    pulsoX = 1000 + ((uint32_t)promedioX * 1000 / 4095);
    pulsoY = 1000 + ((uint32_t)promedioY * 1000 / 4095);

    // Actualiza PWM
    TIM_UpdateMatchValue(LPC_TIM2, 1, pulsoX);
    TIM_UpdateMatchValue(LPC_TIM2, 2, pulsoY);

    // LEDs según eje X
    if(pulsoX < 1365){
        GPIO_SetValue(LED1_PORT, (1<<LED1_PIN));
        GPIO_ClearValue(LED2_PORT, (1<<LED2_PIN));
        GPIO_ClearValue(LED3_PORT, (1<<LED3_PIN));
    } else if(pulsoX < 1730){
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

    // Servos
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    PinCfg.Pinnum = SERVO_X_PIN;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<SERVO_X_PIN), 1);

    PinCfg.Pinnum = SERVO_Y_PIN;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<SERVO_Y_PIN), 1);
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

    // MR1 y MR2 = servos X/Y
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    TIM_MATCHCFG_Type match2 = match1;
    match2.MatchChannel = 2;
    TIM_ConfigMatch(LPC_TIM2, &match2);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// ================== CONFIGURACIÓN ADC ==================
void ConfADC(void) {
    PINSEL_CFG_Type PinCfg;

    // Eje X (P0.25)
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 25;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // Eje Y (P0.26)
    PinCfg.Pinnum = 26;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_ChannelCmd(LPC_ADC, ADC_CH_X, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH_Y, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_ADGINTEN, ENABLE);

    NVIC_EnableIRQ(ADC_IRQn);
}
