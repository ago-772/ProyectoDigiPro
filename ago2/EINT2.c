/*
Guarda en un buffer las posiciones actuales de los 3 servos (MR1–MR3 del Timer2) 
cada vez que se presiona el botón EINT2 (P2.12).
Enciende brevemente el LED rojo (P0.22) para indicar que el movimiento se guardó.
*/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_nvic.h"

#define EINT2_PORT 2
#define EINT2_PIN  12

#define LED_ROJO_PORT  0
#define LED_ROJO_PIN   22

#define MAX_MOV 10     // cantidad de movimientos que guarda el buffer
#define N_SERVOS 3     // 3 servos

volatile uint32_t bufferMov[MAX_MOV][N_SERVOS]; // buffer circular de movimientos
volatile uint8_t indice = 0;

// === Configura LEDs y EINT2 ===
void ConfGPIO(void) {
    PINSEL_CFG_Type PinCfg;

    // LED rojo
    PinCfg.Portnum = LED_ROJO_PORT;
    PinCfg.Pinnum = LED_ROJO_PIN;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(LED_ROJO_PORT, (1 << LED_ROJO_PIN), 1);

    // EINT2 (P2.12)
    PinCfg.Portnum = EINT2_PORT;
    PinCfg.Pinnum = EINT2_PIN;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);
}

// === Configura interrupción externa EINT2 ===
void ConfEINT2(void) {
    EXTI_InitTypeDef extiCfg;
    extiCfg.EXTI_Line = EXTI_EINT2;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&extiCfg);
    EXTI_ClearEXTIFlag(EXTI_EINT2);
    NVIC_EnableIRQ(EINT2_IRQn);
}

// === Interrupción externa ===
void EINT2_IRQHandler(void) {
    EXTI_ClearEXTIFlag(EXTI_EINT2);

    // Guarda las posiciones actuales del Timer2 (MR1, MR2, MR3)
    bufferMov[indice][0] = LPC_TIM2->MR1;
    bufferMov[indice][1] = LPC_TIM2->MR2;
    bufferMov[indice][2] = LPC_TIM2->MR3;

    indice++;
    if (indice >= MAX_MOV) indice = 0; // reinicia el índice si llena el buffer

    GPIO_SetValue(LED_ROJO_PORT, (1 << LED_ROJO_PIN)); // LED indica guardado
    //delay
    GPIO_ClearValue(LED_ROJO_PORT, (1 << LED_ROJO_PIN)); // LED indica guardado
}

// === MAIN ===
int main(void) {
    ConfGPIO();
    ConfEINT2();

    while (1) {
    }
}