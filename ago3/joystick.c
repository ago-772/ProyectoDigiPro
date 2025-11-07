#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"

#define ADC_CH_X 4    // Canal del joystick (ej. P1.30 -> AD0.4)
#define LED1_PORT 0
#define LED1_PIN 22
#define LED2_PORT 3
#define LED2_PIN 25
#define LED3_PORT 3
#define LED3_PIN 26

volatile uint16_t valorADC = 0;

// Configura pines de salida para los LEDs
void ConfGPIO_JoystickTest(void) {
    GPIO_SetDir(LED1_PORT, 1<<LED1_PIN, 1);
    GPIO_SetDir(LED2_PORT, 1<<LED2_PIN, 1);
    GPIO_SetDir(LED3_PORT, 1<<LED3_PIN, 1);
}

// Configura el pin del joystick (ADC)
void ConfADC_JoystickTest(void) {
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;           // P1.30 -> AD0.4
    PinCfg.Funcnum = 3;           // Función ADC
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);    // 200 kHz
    ADC_ChannelCmd(LPC_ADC, ADC_CH_X, ENABLE);
}

// Lee el valor del joystick y actualiza LEDs
void LeerJoystick(void) {
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CH_X, ADC_DATA_DONE)));

    valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH_X);

    if (valorADC < 1365) { // zona izquierda
        GPIO_SetValue(LED1_PORT, 1<<LED1_PIN);
        GPIO_ClearValue(LED2_PORT, 1<<LED2_PIN);
        GPIO_ClearValue(LED3_PORT, 1<<LED3_PIN);
    } 
    else if (valorADC < 2730) { // zona central
        GPIO_SetValue(LED2_PORT, 1<<LED2_PIN);
        GPIO_ClearValue(LED1_PORT, 1<<LED1_PIN);
        GPIO_ClearValue(LED3_PORT, 1<<LED3_PIN);
    } 
    else { // zona derecha
        GPIO_SetValue(LED3_PORT, 1<<LED3_PIN);
        GPIO_ClearValue(LED1_PORT, 1<<LED1_PIN);
        GPIO_ClearValue(LED2_PORT, 1<<LED2_PIN);
    }
}


int main(void) {
ConfGPIO_JoystickTest();
ConfADC_JoystickTest();

while(1) {
    LeerJoystick();
}
}
