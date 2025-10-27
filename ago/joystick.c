/*
JOYSTICK IZQUIERDO
   X → P0.23 (ADC0.0)
   Y → P0.24 (ADC0.1)

JOYSTICK DERECHO
   X → P0.25 (ADC0.2)
   Y → P0.26 (ADC0.3)

se habilitan 4 salidas del Timer2, una para cada servo.
SERVOS (control por PWM)
   Base   → P2.0 (PWM1.1)
   Hombro → P2.1 (PWM1.2)
   Codo   → P2.2 (PWM1.3)
   Garra  → P2.3 (PWM1.4)

5V fuente externa → servos (rojo)
GND común → LPC1769 y servos

 */


//Lee los joysticks → convierte sus señales analógicas (0–3.3 V) en números digitales (0–4095) usando el ADC.
//Convierte esos números en señales PWM de diferente ancho (1 ms ↔ 2 ms).
//Manda esas señales a los servos, para que giren según cómo movés los joysticks.

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"

// Canales ADC (joysticks)
#define SERVO1_CH   1   // PWM1.1
#define SERVO2_CH   2   // PWM1.2
#define SERVO3_CH   3   // PWM1.3
#define SERVO4_CH   4   // PWM1.4

#define ADC_BASE_X   ADC_CHANNEL_0   // P0.23
#define ADC_BASE_Y   ADC_CHANNEL_1   // P0.24
#define ADC_CODO_X   ADC_CHANNEL_2   // P0.25
#define ADC_GARRA_Y  ADC_CHANNEL_3   // P0.26


//------------ configuracion  de las entradas analogicas del joystick ------------
//------------ configuracion  de las salidas, TIMER2 , una para cada servo ------------
void PIN_Config(void) {
    PINSEL_CFG_Type PinCfg;

    // --- ADC (joysticks) ---
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;

    PinCfg.Pinnum = 23; PINSEL_ConfigPin(&PinCfg); // AD0.0
    PinCfg.Pinnum = 24; PINSEL_ConfigPin(&PinCfg); // AD0.1
    PinCfg.Pinnum = 25; PINSEL_ConfigPin(&PinCfg); // AD0.2
    PinCfg.Pinnum = 26; PINSEL_ConfigPin(&PinCfg); // AD0.3

    // === Pines MAT2.0 - MAT2.3 ===
    PinCfg.Funcnum = 3; // función 11 para MAT2.x
    PinCfg.Portnum = 0;

    for (int i = 6; i <= 9; i++) {
        PinCfg.Pinnum = i;
        PINSEL_ConfigPin(&PinCfg);
    }
}

//------------ Configuracion del adc ------------
void ADC_Config(void) {
    ADC_Init(LPC_ADC, 200000); // Frecuencia ADC ≈ 200 kHz
    ADC_ChannelCmd(LPC_ADC, ADC_BASE_X, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_BASE_Y, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CODO_X, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_GARRA_Y, ENABLE);
}

//------------Configuracion del Timer------------
void TIMER2_Config(void) {
    TIM_TIMERCFG_Type TimerCfg;
    TIM_MATCHCFG_Type MatchCfg;

    // --- Timer en modo TIMER ---
    TimerCfg.PrescaleOption = TIM_PRESCALE_USVAL; // cuenta en microsegundos
    TimerCfg.PrescaleValue  = 1;                   // 1 tick = 1 µs
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TimerCfg);

    // --- MATCH0: período (20 ms) ---
    MatchCfg.MatchChannel = 0;
    MatchCfg.IntOnMatch   = DISABLE;
    MatchCfg.StopOnMatch  = DISABLE;
    MatchCfg.ResetOnMatch = ENABLE;
    MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    MatchCfg.MatchValue   = 20000; // 20 ms = 50 Hz
    TIM_ConfigMatch(LPC_TIM2, &MatchCfg);

    // --- MATCH1..3: servos ---
    for (int i = 1; i <= 3; i++) {
        MatchCfg.MatchChannel = i;
        MatchCfg.IntOnMatch   = DISABLE;
        MatchCfg.StopOnMatch  = DISABLE;
        MatchCfg.ResetOnMatch = DISABLE;
        MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE; // alterna salida
        MatchCfg.MatchValue   = 1500; // valor inicial (1.5 ms)
        TIM_ConfigMatch(LPC_TIM2, &MatchCfg);
    }

    TIM_Cmd(LPC_TIM2, ENABLE); // arranca timer
}

void Mover_Servo(uint8_t canal, uint16_t valorADC) {
    uint32_t pulso = 1000 + (valorADC * 1000) / 4095; // 1000–2000 µs
    TIM_UpdateMatchValue(LPC_TIM2, canal, pulso);
}

void TIMER2_IRQHandler(void) {
    static uint8_t canal = 0;
    uint16_t valorADC;

	// Deshabilitás todos los canales
	for (int i = 0; i < 4; i++) ADC_ChannelCmd(LPC_ADC, i, DISABLE);

	// Habilitás solo el canal actual
	ADC_ChannelCmd(LPC_ADC, canal, ENABLE);

	// Iniciás conversión
	ADC_StartCmd(LPC_ADC, ADC_START_NOW);

	// Esperás a que termine ese canal
	while (!(ADC_ChannelGetStatus(LPC_ADC, canal, ADC_DATA_DONE)));
	valorADC = ADC_ChannelGetData(LPC_ADC, canal);

	// Actualizás el servo correspondiente
	switch (canal) {
	    case 0: Mover_Servo(1, valorADC); break; // Base
	    case 1: Mover_Servo(2, valorADC); break; // Hombro
	    case 2: Mover_Servo(3, valorADC); break; // Codo
	    case 3: Mover_Servo(4, valorADC); break; // Garra
	}

	// Avanzás al siguiente canal
	canal = (canal + 1) % 4;

}



int main(void) {
    SystemInit();
    PIN_Config();
    ADC_Config();
    TIMER2_Config();

    while(1) {
        
    }
}