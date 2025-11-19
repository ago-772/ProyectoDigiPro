/*
 * Proyecto: Servo + DMA Inteligente (RAM -> PWM Match)
 * MCU: LPC1769
 * Modos:
 * 0: Manual (ADC -> Servo) + Grabación en Buffer
 * 1: Reproducción (DMA -> Timer Match). CPU Libre.
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_exti.h"

// ---------------------------------------------------------------------------
// Definiciones
// ---------------------------------------------------------------------------
#define SERVO_PIN   9           // P0.9
#define LED_PIN     22          // P0.22 (Indicador de modo)
#define ADC_CH      4           // AD0.4 (P1.30)
#define BUFFER_SIZE 20         // Cantidad de movimientos a grabar

// Conexión DMA para el Match 0 del Timer 2 (Ver User Manual Tabla GPDMA)
#define GPDMA_CONN_TIM2_MR0  12

// ---------------------------------------------------------------------------
// Variables globales
// ---------------------------------------------------------------------------
volatile uint16_t valorADC = 0;
volatile uint32_t pulso = 1500;

uint32_t pulseBuffer[BUFFER_SIZE] = {
    1000, 1100, 1200, 1300, 1400,
    1500, 1600, 1700, 1800, 1900,
    2000, 1900, 1800, 1700, 1600,
    1500, 1500, 1500, 1500, 1500 // Secuencia de prueba
};

volatile uint32_t modo = 0;      // 0 = Grabar/Manual, 1 = Reproducir DMA
volatile uint32_t indiceRec = 0; // Indice para grabar

// ---------------------------------------------------------------------------
// Configuración GPIO
// ---------------------------------------------------------------------------
void ConfGPIO(void)
{
	PINSEL_CFG_Type PinCfg;

	// Configuración general para GPIO
	PinCfg.Funcnum = 0;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

	// Servo 1 (P0.9)
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = SERVO_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, (1 << SERVO_PIN), 1);

	// LED (P0.22)
	PinCfg.Pinnum = LED_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, (1 << LED_PIN), 1);

	// Inicia en modo Manual/Stop (LED apagado)
	GPIO_ClearValue(0,1<<LED_PIN);
}

// ---------------------------------------------------------------------------
// Configuración TIMER2 (PWM)
// ---------------------------------------------------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1; // 1 tick = 1 microsegundo
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // --- MATCH0: periodo 20 ms (20000 us) ---
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // --- MATCH1: Servo 1 ---
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500; // Valor inicial
    TIM_ConfigMatch(LPC_TIM2, &match1);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// ---------------------------------------------------------------------------
// Interrupción Timer2 (PWM software)
// ---------------------------------------------------------------------------

void TIMER2_IRQHandler(void)
{
    // MATCH0 → reinicia ciclo PWM, pone el pin en ALTO
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1 << SERVO_PIN));
    }

    // MATCH1 → fin pulso servo 1, pone el pin en BAJO
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1 << SERVO_PIN));
    }
}

// ---------------------------------------------------------------------------
// Configuración DMA (Modo Reproducción)
// ---------------------------------------------------------------------------
void StartDMA_Playback(void)
{
    GPDMA_Channel_CFG_Type dmaCfg;

    GPDMA_Init();

    dmaCfg.ChannelNum    = 0;
    dmaCfg.SrcMemAddr    = (uint32_t)pulseBuffer;
    dmaCfg.DstMemAddr    = (uint32_t)&(LPC_TIM2->MR1);
    dmaCfg.TransferSize  = BUFFER_SIZE;
    dmaCfg.TransferWidth = 0;

    dmaCfg.TransferType  = GPDMA_TRANSFERTYPE_M2P;

    dmaCfg.DstConn 		 = GPDMA_CONN_MAT2_0; 		// Disparado por Timer2 Match 0
    dmaCfg.SrcConn       = 0;

    dmaCfg.DMALLI        = 0; 						// Sin listas enlazadas


/*
// Preparar LLI (Linked List Item) para transferencia circular
    dmaLLI.SrcAddr = (uint32_t)	pulsoBuffer;
    dmaLLI.DstAddr = (uint32_t) &(LPC_TIM2->MR1); // Destino: Match Register 1 del Timer 2
    dmaLLI.NextLLI = (uint32_t)	&dmaLLI; // Bucle: apunta a sí mismo (Circular)
    dmaLLI.Control =
          (BUFFER_SIZE) 					// Bits 0–11: TransferSize
        | (2 << 18)							// SWidth = 2 → 32-bit
        | (2 << 21) 						// DWidth = 2 → 32-bit
        | (1 << 26) 						// SI = 1 → Source Increment enable
        | (0 << 27); 						// DI = 0 → Dest Increment disable

 */
    GPDMA_Setup(&dmaCfg);
}

// ---------------------------------------------------------------------------
// ADC (Modo Manual)
// ---------------------------------------------------------------------------
void ConfADC(void)
{
	// P1.30 (AD0.4)
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum  = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_BurstCmd(LPC_ADC, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);

    NVIC_EnableIRQ(ADC_IRQn);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
    //NVIC_EnableIRQ(ADC_IRQn);

}

void ADC_IRQHandler(void)
{
    // Solo se ejecuta si estamos en modo MANUAL (0)
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        // Cálculo Matemático (Mapeo 0-4095 a 1000-2000)
        if (valorADC >= 1948 && valorADC < 2147) {
            pulso = 1500;
        } else if (valorADC < 1948) {
            pulso = 1000 + (uint32_t)(((float)valorADC / 1947.0f) * 500.0f);
        } else {
            pulso = 1500 + (uint32_t)(((float)(valorADC - 2147) / 1948.0f) * 500.0f);
        }

        // Actualiza Match Register del Timer2 directamente
        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }
}

// ---------------------------------------------------------------------------
// EINT1 - Cambio de Modo
// ---------------------------------------------------------------------------
void ConfEINT1(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 11;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    GPIO_SetDir(2, (1 << 11), 0);

    EXTI_InitTypeDef extiCfg;
    extiCfg.EXTI_Line = EXTI_EINT1;
    extiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    extiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&extiCfg);
    EXTI_ClearEXTIFlag(EXTI_EINT1);
    NVIC_EnableIRQ(EINT1_IRQn);
}

// ---------------------------------------------------------------------------
// Interrupción EINT0 (Manejador de estado)
// ---------------------------------------------------------------------------

void EINT1_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT1);
    for(volatile int i=0; i<100000; i++); // Anti-rebote

    if (modo == 0)
    {
        // ---------------------------------------
        // PASAR A REPRODUCCIÓN
        // ---------------------------------------
        modo = 1;

        GPIO_SetValue(0, (1<<LED_PIN));			// Encender Led

        StartDMA_Playback();       				// configuración del DMA

        NVIC_DisableIRQ(ADC_IRQn);				// Deshabilitar ADC
        ADC_IntConfig(LPC_ADC, ADC_CH, DISABLE);// Deshabilitar ADC

        GPDMA_ChannelCmd(0, ENABLE);			// Habilitar canal DMA

        // IMPORTANTE: Habilitar que el Timer2 pida DMA cuando haga Match 0
        // Bit 3 del MCR es "DMAMR0" (DMA on Match 0)
        LPC_TIM2->MCR |= (1 << 3);				//??????????????????????????

    }
    else
    {
        // ---------------------------------------
        // PASAR A MANUAL
        // ---------------------------------------
        modo = 0;
        indiceRec = 0; 							// Resetear indice

        GPIO_SetValue(0, (1<<LED_PIN));			// Apagar Led

        GPDMA_ChannelCmd(0, DISABLE);			// Detener DMA

        ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);	// Habilitar ADC
        NVIC_EnableIRQ(ADC_IRQn);

        // Desactivar la petición de DMA en el Timer (Bit 3 a 0)
        // Si no lo hacemos, el Timer seguirá gritando "DMA!" sin que nadie escuche
        LPC_TIM2->MCR &= ~(1 << 3);				//??????????????????????
    }
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(void)
{
    ConfGPIO();
    GPIO_SetValue(0, (1<<LED_PIN)); // LED ON = Modo Manual

    ConfTIMER2(); // Inicia el PWM
    ConfADC();    // Inicia el ADC
    ConfEINT1();  // Botón

    while (1)
    {

    }
}
