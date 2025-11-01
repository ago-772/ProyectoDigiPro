/*
guarda movimientos de los servos y los reproduce automáticamente con DMA.
Con el botón en P2.12 (EINT2) se guardan los valores actuales de los 3 servos en un buffer.
Con el botón en P2.13 (EINT3) se inicia o detiene la reproducción automática de esos movimientos, 
que el DMA copia al Timer2 sin usar la CPU.
*/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_timer.h"

#define EINT2_PORT 2
#define EINT2_PIN  12   // Guardar movimientos
#define EINT3_PORT 2
#define EINT3_PIN  13   // Iniciar/Detener DMA

#define MAX_MOV   10
#define N_SERVOS  3

volatile uint32_t bufferMov[MAX_MOV][N_SERVOS];
volatile uint8_t indiceGuardar = 0;
volatile uint8_t indiceDMA = 0;
volatile uint8_t dmaActivo = 0; // flag de ejecución automática

// CONFIGURACIÓN DE PINES
void ConfGPIO(void) {
    PINSEL_CFG_Type PinCfg;

    // ---- EINT2 (guardar) ----
    PinCfg.Portnum = EINT2_PORT;
    PinCfg.Pinnum = EINT2_PIN;
    PinCfg.Funcnum = 1; // EINT2
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);

    // ---- EINT3 (start/stop) ----
    PinCfg.Portnum = EINT3_PORT;
    PinCfg.Pinnum = EINT3_PIN;
    PinCfg.Funcnum = 1; // EINT3
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);
}

// CONFIGURACIÓN DE INTERRUPCIONES EXTERNAS
void ConfEINT(void) {
    EXTI_InitTypeDef extiCfg;

    // EINT2 - guarda movimientos
    extiCfg.EXTI_Line = EXTI_EINT2;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&extiCfg);
    EXTI_ClearEXTIFlag(EXTI_EINT2);
    NVIC_EnableIRQ(EINT2_IRQn);

    // EINT3 - start/stop DMA
    extiCfg.EXTI_Line = EXTI_EINT3;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&extiCfg);
    EXTI_ClearEXTIFlag(EXTI_EINT3);
    NVIC_EnableIRQ(EINT3_IRQn);
}

// CONFIGURACIÓN DEL DMA
void ConfigDMA(void) {
    GPDMA_Init();

    GPDMA_Channel_CFG_Type dmaCfg;
    dmaCfg.ChannelNum = 0;
    dmaCfg.SrcMemAddr = (uint32_t)&bufferMov[indiceDMA][0];
    dmaCfg.DstMemAddr = (uint32_t)&(LPC_TIM2->MR0);
    dmaCfg.TransferSize = N_SERVOS;
    dmaCfg.TransferWidth = GPDMA_WIDTH_WORD;
    dmaCfg.TransferType = GPDMA_TRANSFERTYPE_M2M;
    dmaCfg.SrcConn = 0;
    dmaCfg.DstConn = 0;
    dmaCfg.DMALLI = 0;

    GPDMA_Setup(&dmaCfg);
    NVIC_EnableIRQ(DMA_IRQn);
}

// EINT2 - Guarda movimientos
void EINT2_IRQHandler(void) {
    EXTI_ClearEXTIFlag(EXTI_EINT2);

    bufferMov[indiceGuardar][0] = LPC_TIM2->MR1;
    bufferMov[indiceGuardar][1] = LPC_TIM2->MR2;
    bufferMov[indiceGuardar][2] = LPC_TIM2->MR3;

    indiceGuardar++;
    if (indiceGuardar >= MAX_MOV) indiceGuardar = 0;
}

// EINT3 - Activa o detiene DMA (start/stop)
void EINT3_IRQHandler(void) {
    EXTI_ClearEXTIFlag(EXTI_EINT3);

    if (!dmaActivo) {
        dmaActivo = 1;
        GPDMA_ChannelCmd(0, ENABLE);  // Comienza reproducción
    } else {
        dmaActivo = 0;
        GPDMA_ChannelCmd(0, DISABLE); // Detiene DMA
    }
}

// DMA_IRQHandler - pasa al siguiente movimiento
void DMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) {
        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);

        if (dmaActivo) {
            indiceDMA++;
            if (indiceDMA >= MAX_MOV) indiceDMA = 0;

            // Actualiza fuente del DMA
            GPDMA_Channel_CFG_Type dmaCfg;
            dmaCfg.ChannelNum = 0;
            dmaCfg.SrcMemAddr = (uint32_t)&bufferMov[indiceDMA][0];
            dmaCfg.DstMemAddr = (uint32_t)&(LPC_TIM2->MR0);
            dmaCfg.TransferSize = N_SERVOS;
            dmaCfg.TransferWidth = GPDMA_WIDTH_WORD;
            dmaCfg.TransferType = GPDMA_TRANSFERTYPE_M2M;
            dmaCfg.SrcConn = 0;
            dmaCfg.DstConn = 0;
            dmaCfg.DMALLI = 0;

            GPDMA_Setup(&dmaCfg);
            GPDMA_ChannelCmd(0, ENABLE);
        }
    }
}

// MAIN
int main(void) {
    ConfGPIO();
    ConfEINT();
    ConfigDMA();

    while (1) {
        // bucle vacío, DMA y EINTs hacen todo
    }
}