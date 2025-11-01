//conf DMA

#include "lpc17xx_gpdma.h"
#include "lpc17xx_timer.h"

#define N_SERVOS 3
#define MAX_MOV 10

extern volatile uint32_t bufferMov[][N_SERVOS]; // declarado en el otro código
volatile uint8_t indiceDMA = 0;

// === Configura el DMA ===
void ConfDMA(void) {
    GPDMA_Init(); // Inicializa el controlador DMA

    GPDMA_Channel_CFG_Type dmaCfg;
    dmaCfg.ChannelNum = 0;               // Canal 0 del DMA
    dmaCfg.SrcMemAddr = (uint32_t)&bufferMov[indiceDMA][0]; // fuente: buffer de servos
    dmaCfg.DstMemAddr = (uint32_t)&(LPC_TIM2->MR0);          // destino: primer MR del Timer2
    dmaCfg.TransferSize = N_SERVOS;      // cantidad de transferencias (3)
    dmaCfg.TransferWidth = GPDMA_WIDTH_WORD;
    dmaCfg.TransferType = GPDMA_TRANSFERTYPE_M2M; // memoria a memoria
    dmaCfg.SrcConn = 0;
    dmaCfg.DstConn = 0;
    dmaCfg.DMALLI = 0;

    GPDMA_Setup(&dmaCfg);
    NVIC_EnableIRQ(DMA_IRQn); // habilita interrupción DMA
}

// === Activa el DMA manualmente (por ejemplo, tras presionar otro botón) ===
void IniciarDMA(void) {
    GPDMA_ChannelCmd(0, ENABLE);
}

// === Interrupción del DMA ===
void DMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0)) {
        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);

        // Mueve el índice al siguiente movimiento
        indiceDMA++;
        if (indiceDMA >= MAX_MOV) indiceDMA = 0;

        // Reconfigura la fuente para el próximo movimiento
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
        GPDMA_ChannelCmd(0, ENABLE); // reinicia la transferencia
    }
}
