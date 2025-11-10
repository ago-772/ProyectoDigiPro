#include "lpc17xx_pinsel.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_nvic.h"
#include <string.h>

#define EINT2_PORT 2
#define EINT2_PIN  12

#define LED_ROJO_PORT  0
#define LED_ROJO_PIN   22

#define MAX_MOV 3
#define N_SERVOS 3

volatile uint32_t bufferMov[MAX_MOV][N_SERVOS];
volatile uint8_t indice = 0;

// Prototipos
void configGPIO_LED_EINT2(void);
void configGPIO_UART(void);
void configUART(void);
void configEINT2(void);
void ResetMovimientos(void);

// MAIN
int main(void) {
    configGPIO_LED_EINT2();
    configGPIO_UART();
    configUART();
    configEINT2();
   // ResetMovimientos(); // Mensaje en pantalla: Movimientos borrados

    while(1) {
        // Nada aquí. Todo sucede por interrupción.
    }

    return 0;
}

// CONFIGURACIÓN DE PINES: LED + BOTÓN EINT2
void configGPIO_LED_EINT2(void) {
    PINSEL_CFG_Type pin;

    // LED (P0.22)
    pin.Portnum = LED_ROJO_PORT;
    pin.Pinnum = LED_ROJO_PIN;
    pin.Funcnum = 0;
    pin.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&pin);
    GPIO_SetDir(LED_ROJO_PORT, (1<<LED_ROJO_PIN), 1);

    // EINT2 (P2.12)
    pin.Portnum = EINT2_PORT;
    pin.Pinnum = EINT2_PIN;
    pin.Funcnum = 1;
    pin.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&pin);
}

// CONFIGURACIÓN UART (MISMA ESTRUCTURA QUE TU EJEMPLO)
void configGPIO_UART(void) {
    PINSEL_CFG_Type psel_conf;

    psel_conf.Portnum = 0;
    psel_conf.Pinmode = PINSEL_PINMODE_TRISTATE;
    psel_conf.OpenDrain = PINSEL_PINMODE_NORMAL;

    // TXD0 = P0.2
    psel_conf.Pinnum = 2;
    psel_conf.Funcnum = 1;
    PINSEL_ConfigPin(&psel_conf);

    // RXD0 = P0.3
    psel_conf.Pinnum = 3;
    psel_conf.Funcnum = 1;
    PINSEL_ConfigPin(&psel_conf);
}

void configUART(void) {
    UART_CFG_Type uart_cfg;
    UART_FIFO_CFG_Type fifo_cfg;

    UART_ConfigStructInit(&uart_cfg);
    UART_Init((LPC_UART_TypeDef *)LPC_UART0, &uart_cfg);

    UART_FIFOConfigStructInit(&fifo_cfg);
    UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &fifo_cfg);

    UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);
}

// CONFIGURACIÓN INTERRUPCIÓN EXTERNA
void configEINT2(void) {
    EXTI_InitTypeDef extiCfg;
    extiCfg.EXTI_Line = EXTI_EINT2;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&extiCfg);
    EXTI_ClearEXTIFlag(EXTI_EINT2);
    NVIC_EnableIRQ(EINT2_IRQn);
}

// ISR: GUARDAR MOVIMIENTO + MENSAJE UART
void EINT2_IRQHandler(void) {
    EXTI_ClearEXTIFlag(EXTI_EINT2);

    // Guardar MR1, MR2, MR3
    bufferMov[indice][0] = LPC_TIM2->MR1;
    bufferMov[indice][1] = LPC_TIM2->MR2;
    bufferMov[indice][2] = LPC_TIM2->MR3;

    // LED breve
    GPIO_SetValue(LED_ROJO_PORT, (1<<LED_ROJO_PIN));
    for(volatile int i=0;i<70000;i++);
    GPIO_ClearValue(LED_ROJO_PORT, (1<<LED_ROJO_PIN));

    // Mensajes
    // Enviar el mensaje en modo "bloqueante"
    if(indice == 0)
        UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t*)"Primer movimiento guardado\r\n", 29, BLOCKING);
    else if(indice == 1)
        UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t*)"Segundo movimiento guardado\r\n", 30, BLOCKING);
    else if(indice == 2)
        UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t*)"Tercer movimiento guardado\r\n", 29, BLOCKING);

    indice++;
    if(indice >= MAX_MOV) indice = 0;
}

// RESET
void ResetMovimientos(void){
    for(int i=0;i<MAX_MOV;i++){
        bufferMov[i][0] = bufferMov[i][1] = bufferMov[i][2] = 0;
    }
    indice = 0;
    UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t*)"Movimientos borrados\r\n", 22, BLOCKING);
}