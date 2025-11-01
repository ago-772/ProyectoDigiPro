/*
EINT1 (P2.11) para borrar todo el buffer de movimientos 
y reiniciar los índices de guardado y reproducción
*/
#include "lpc17xx_pinsel.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_nvic.h"

#define EINT1_PORT 2
#define EINT1_PIN  11

#define MAX_MOV 10
#define N_SERVOS 3

extern volatile uint32_t bufferMov[MAX_MOV][N_SERVOS];
extern volatile uint8_t indiceGuardar, indiceDMA;

void ConfEINT1(void) {
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = EINT1_PORT;
    PinCfg.Pinnum = EINT1_PIN;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);

    EXTI_InitTypeDef extiCfg;
    extiCfg.EXTI_Line = EXTI_EINT1;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&extiCfg);
    EXTI_ClearEXTIFlag(EXTI_EINT1);
    NVIC_EnableIRQ(EINT1_IRQn);
}

void EINT1_IRQHandler(void) {
    EXTI_ClearEXTIFlag(EXTI_EINT1);

    for (int i = 0; i < MAX_MOV; i++)
        for (int j = 0; j < N_SERVOS; j++)
            bufferMov[i][j] = 0; // limpia todo

    indiceGuardar = 0;
    indiceDMA = 0;
}
