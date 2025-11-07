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

#define SERVO_MIN_US 1000   // 1.0 ms  -> ángulo mínimo
#define SERVO_MAX_US 2000   // 2.0 ms  -> ángulo máximo

volatile uint16_t valorADC = 0;


#define N_MUESTRAS 20
volatile uint16_t bufferADC[N_MUESTRAS];
volatile uint8_t indiceADC = 0;
volatile uint32_t pulso = 1500;   // valor inicial seguro

//  INTERRUPCIÓN ADC 
void ADC_IRQHandler(void) {
    // Espera fin de conversión
    while(!ADC_ChannelGetStatus(LPC_ADC, ADC_CH, ADC_DATA_DONE));

    // Lee valor del canal
    valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH);

    // Guarda muestra en buffer circular
    bufferADC[indiceADC++] = valorADC;
    if (indiceADC >= N_MUESTRAS) indiceADC = 0;

    // Calcula promedio cada 20 lecturas
    uint32_t suma = 0;
    for (int i = 0; i < N_MUESTRAS; i++)
        suma += bufferADC[i];
    uint16_t promedio = suma / N_MUESTRAS;

    // Mapea 0–4095 → 1000–2000 µs con el promedio
    pulso = 1000 + ((uint32_t)promedio * 1000 / 4095);

    // Actualiza el valor del MATCH2 (ancho de pulso)
    TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);

    // Lógica de LEDs según el pulso actual
    if(pulso >= 0 && pulso < 1365){
        GPIO_SetValue(0, (1 << 22));
        GPIO_ClearValue(3 , (1 << 25));
        GPIO_ClearValue(3 , (1 << 26));
    }else if(pulso >= 1365 && pulso < 2730){
        GPIO_SetValue(3, (1 << 25));
        GPIO_ClearValue(0 , (1 << 22));
        GPIO_ClearValue(3 , (1 << 26));
    }else if(pulso >= 2730 && pulso < 4095){
        GPIO_SetValue(3, (1 << 26));
        GPIO_ClearValue(0 , (1 << 22));
        GPIO_ClearValue(3 , (1 << 25));
    }
}

// INTERRUPCIÓN TIMER0
void TIMER0_IRQHandler(void) {
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)) {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
        ADC_StartCmd(LPC_ADC, ADC_START_NOW);  // Inicia una conversión manual
    }
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

//  CONFIGURACIÓN GPIO
void ConfGPIO(void)
{
	//configuracion pin P0.9
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinnum = SERVO_PIN;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    GPIO_SetDir(0,1<<22,1);
    GPIO_SetDir(3,1<<25,1);
    GPIO_SetDir(3,1<<26,1);

    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<SERVO_PIN), 1);					//configurar como salida
}

//  CONFIGURACIÓN TIMER2 
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;

    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MR0 = periodo (20 ms)
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;							
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // MR1 = duty inicial
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

//  CONFIGURACIÓN TIMER0 
void ConfTIMER0(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1000;   // 1 tick = 1 ms

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 10;          // 10 ms
    TIM_ConfigMatch(LPC_TIM0, &match0);

    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

//  CONFIGURACIÓN ADC 
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

    ADC_Init(LPC_ADC, 200000);           // 200 kHz
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    // --- MODO BURST DESACTIVADO ---
    NVIC_EnableIRQ(ADC_IRQn);
}

int main(void)
{
    ConfGPIO();
    ConfTIMER2();
    ConfTIMER0();  
    ConfADC();

    while (1);
}
