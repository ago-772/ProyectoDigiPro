/**********************************************************************
* $Id$      dac_dma_circular.c
* @brief    DAC DMA Circular Buffer Example (No CPU intervention)
**********************************************************************/
#include "lpc17xx_dac.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpdma.h"

/*------------------------- DEFINES ------------------------------*/
#define NUM_SAMPLES     1023      // Tamaño del array (onda)

/*------------------------- GLOBAL VARIABLES ---------------------*/
// Array que contiene la onda a reproducir
uint32_t dac_buffer[NUM_SAMPLES];

// Estructura para la Lista Enlazada (Linked List Item) del DMA
GPDMA_LLI_Type DMA_LLI_Struct;

// Configuración del canal DMA
GPDMA_Channel_CFG_Type GPDMACfg;

void Config_Pins(void);
void Config_DAC_And_Fill_Buffer(void); // Llena el array y configura DAC
void Config_DMA_Circular(void);        // Configura DMA en modo circular
void System_Init(void);

//------------------------- CONFIG DAC -------------------------

/**
 * DAC P0.26 como salida analógica
 */
void Config_Pins(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 26;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
}

//------------------------- CONFIG DAC -------------------------

void Config_DAC_And_Fill_Buffer(void)
{
    uint32_t i;
    DAC_CONVERTER_CFG_Type DAC_ConverterConfigStruct;

    // Generamos una rampa (diente de sierra) de 0 a 1023
    for(i = 0; i < NUM_SAMPLES; i++)
    {
        // Escalamos i para que el valor máximo sea 1023 (10 bits)
        uint32_t val = (i * 1023) / NUM_SAMPLES;

        // El hardware del DAC espera el valor en los bits 6 a 15.
        // También ponemos el bit de BIAS (bit 16) en 0 para max update rate.
        dac_buffer[i] = (val << 6);
    }

    /* --- 2. Configurar Hardware DAC --- */
    DAC_ConverterConfigStruct.CNT_ENA = SET;
    DAC_ConverterConfigStruct.DMA_ENA = SET;

    DAC_Init(LPC_DAC);

    // Configura la velocidad de actualización (timeout del contador interno del DAC)
    // Cuanto menor sea este número, más rápida será la frecuencia de la onda.
    DAC_SetDMATimeOut(LPC_DAC, 0x0FFF);

    DAC_ConfigDAConverterControl(LPC_DAC, &DAC_ConverterConfigStruct);
}

//------------------------- CONFIG DMA -------------------------

void Config_DMA_Circular(void)
{
    GPDMA_Init();

    /* --- Configuración de la Lista Enlazada --- */

    DMA_LLI_Struct.SrcAddr = (uint32_t)dac_buffer;    // Origen: El array
    DMA_LLI_Struct.DstAddr = (uint32_t)&(LPC_DAC->DACR); // Destino: Registro DAC
    DMA_LLI_Struct.NextLLI = (uint32_t)&DMA_LLI_Struct;  // <--- SE APUNTA A SÍ MISMA (Circular)

    DMA_LLI_Struct.Control = NUM_SAMPLES
                           | (2 << 18)  // Source width 32 bit
                           | (2 << 21)  // Dest width 32 bit
                           | (1 << 26)  // Source Increment (SI) = TRUE
                           ;

    /* --- Configuración del Canal 0 --- */
    GPDMACfg.ChannelNum = 0;
    GPDMACfg.SrcMemAddr = (uint32_t)dac_buffer;
    GPDMACfg.DstMemAddr = 0; // Se ignora en M2P si usas LLI interno, pero buena práctica
    GPDMACfg.TransferSize = NUM_SAMPLES;
    GPDMACfg.TransferWidth = 0; // Se define en el LLI
    GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memoria a Periférico
    GPDMACfg.SrcConn = 0;
    GPDMACfg.DstConn = GPDMA_CONN_DAC; // Trigger por el DAC

    GPDMACfg.DMALLI = (uint32_t)&DMA_LLI_Struct;

    GPDMA_Setup(&GPDMACfg);

    GPDMA_ChannelCmd(0, ENABLE);
}

/*------------------------- SYSTEM LOGIC -------------------------*/

void System_Init(void)
{
    Config_Pins();
    Config_DAC_And_Fill_Buffer();
    Config_DMA_Circular();
}

/*------------------------- MAIN FUNCTION ------------------------*/
int main(void)
{
    System_Init();

    while (1)
    {
    }

    return 0;
}
