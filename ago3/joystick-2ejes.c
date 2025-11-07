#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_nvic.h"

#define ADC_X 2   // P0.25 -> AD0.2
#define ADC_Y 3   // P0.26 -> AD0.3

#define SERVO1_PIN 9    // Eje X → Servo 1 (P0.9)
#define SERVO2_PIN 8    // Eje Y → Servo 2 (P0.8)

// LEDs de la placa LPCXpresso
#define LED1_PORT 0
#define LED1_PIN 22
#define LED2_PORT 3
#define LED2_PIN 25
#define LED3_PORT 3
#define LED3_PIN 26

volatile uint16_t valorX = 0;
volatile uint16_t valorY = 0;

// ---------- Configuración ADC ----------
void ConfADC_Joystick(void) {
    PINSEL_CFG_Type PinCfg;

    // --- EJE X (P0.25 -> AD0.2) ---
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 25;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // --- EJE Y (P0.26 -> AD0.3) ---
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 26;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_ChannelCmd(LPC_ADC, ADC_X, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_Y, ENABLE);
}

// ---------- Configuración GPIO ----------
void ConfGPIO_LedsServos(void) {
    PINSEL_CFG_Type PinCfg;

    // LEDs
    GPIO_SetDir(LED1_PORT, 1<<LED1_PIN, 1);
    GPIO_SetDir(LED2_PORT, 1<<LED2_PIN, 1);
    GPIO_SetDir(LED3_PORT, 1<<LED3_PIN, 1);

    // Servos como salida GPIO (PWM por software)
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = SERVO1_PIN;
    PinCfg.Funcnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, 1<<SERVO1_PIN, 1);

    PinCfg.Pinnum = SERVO2_PIN;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, 1<<SERVO2_PIN, 1);
}

// ---------- Lee los ejes ----------
void LeerJoystick(void) {
    // Lee eje X
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_X, ADC_DATA_DONE)));
    valorX = ADC_ChannelGetData(LPC_ADC, ADC_X);

    // Lee eje Y
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_Y, ADC_DATA_DONE)));
    valorY = ADC_ChannelGetData(LPC_ADC, ADC_Y);
}

// ---------- Convierte valor ADC a pulso servo ----------
uint32_t MapServo(uint16_t valorADC) {
    return 1000 + ((uint32_t)valorADC * 1000 / 4095);
}

// ---------- Lógica de LEDs según dirección ----------
void MostrarDireccion(uint16_t x, uint16_t y) {
    // Centro
    if (x > 1500 && x < 2600 && y > 1500 && y < 2600) {
        GPIO_SetValue(LED2_PORT, 1<<LED2_PIN);
        GPIO_ClearValue(LED1_PORT, 1<<LED1_PIN);
        GPIO_ClearValue(LED3_PORT, 1<<LED3_PIN);
    }
    // Izquierda o derecha
    else if (x < 1500) {   // Izquierda
        GPIO_SetValue(LED1_PORT, 1<<LED1_PIN);
        GPIO_ClearValue(LED2_PORT, 1<<LED2_PIN);
        GPIO_ClearValue(LED3_PORT, 1<<LED3_PIN);
    } else if (x > 2600) { // Derecha
        GPIO_SetValue(LED3_PORT, 1<<LED3_PIN);
        GPIO_ClearValue(LED2_PORT, 1<<LED2_PIN);
        GPIO_ClearValue(LED1_PORT, 1<<LED1_PIN);
    }
}


int main(void) {
    ConfGPIO_LedsServos();
    ConfADC_Joystick();

    while (1) {
        LeerJoystick();

        uint32_t pulsoX = MapServo(valorX);
        uint32_t pulsoY = MapServo(valorY);

        // Muestra dirección con LEDs
        MostrarDireccion(valorX, valorY);

        // En esta parte podrías actualizar el PWM de los servos:
        // TIM_UpdateMatchValue(LPC_TIM2, 1, pulsoX);
        // TIM_UpdateMatchValue(LPC_TIM2, 2, pulsoY);
    }
}
