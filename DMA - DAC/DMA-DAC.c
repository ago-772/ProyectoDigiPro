/*
 * Proyecto: Grabadora Servo Simplificada (CPU graba, DMA reproduce)
 * MCU: LPC1769
 *
 * Flujo de Datos:
 * 1. MANUAL: ADC -> IRQ (Calcula Pulso) -> Servo.
 * 2. REC:    ADC -> IRQ (Calcula Pulso) -> Servo + Guardado en Array[i].
 * 3. PLAY:   Array -> DMA (M2P) -> Timer2 MR1 (Trigger: Timer2 PWM Match).
 *
 * TIENE DAC
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_exti.h"

#include "lpc17xx_dac.h"

// --- Definiciones ---
#define SERVO_PIN       9    // P0.9
#define LED_REC         22   // P0.22
#define ADC_CH          4    // P1.30

#define PWM_PERIOD_US   20000    // 20ms
#define BUFFER_SIZE     2000     // Tamaño del buffer

// --- Variables Globales ---
volatile uint16_t valorADC1 = 0;
volatile uint32_t pulso = 1500;

// Buffer en RAM (Guarda valores ya cocinados: 1000 a 2000)
uint32_t servo_buffer[BUFFER_SIZE];
volatile uint32_t write_index = 0; // Índice para escribir en el buffer

volatile uint8_t systemMode = 0;   // 0: Manual, 1: Rec, 2: Play

// Lista Enlazada para Playback
GPDMA_LLI_Type LLI_Play;

// --- Configuración GPIO ---
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    PinCfg.Pinnum = LED_REC;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<LED_REC), 1);
}
//--------------------------------DAC
void ConfDAC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 26;
    PinCfg.Funcnum = 2; // AOUT (Salida Analógica)
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    DAC_Init(LPC_DAC);
}


// --- Configuración ADC ---
void ConfADC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3; // AD0.4
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_BurstCmd(LPC_ADC, ENABLE); // Burst: Conversión continua

    NVIC_EnableIRQ(ADC_IRQn);
}

// --- Configuración Timer2 (PWM) ---
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MATCH 0: Periodo (20ms)
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = PWM_PERIOD_US;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // IMPORTANTE: Habilitar DMA Request en Match 0 (Bit 3 de MCR)
    // Esto es vital para que el DMA funcione en modo Playback
    LPC_TIM2->MCR |= (1 << 3);

    // MATCH 1: Ancho del Pulso (Servo)
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// --- DMA Config: SOLO PLAYBACK (Memoria -> Timer) ---
void SetupDMA_Play(void)
{
    // Configurar LLI para bucle infinito (Circular)
    LLI_Play.SrcAddr = (uint32_t)servo_buffer;
    LLI_Play.DstAddr = (uint32_t)&(LPC_TIM2->MR1); // Destino: Registro del Timer
    LLI_Play.NextLLI = (uint32_t)&LLI_Play;        // Se apunta a sí misma
    LLI_Play.Control = BUFFER_SIZE
                     | (2 << 18) // Source Width 32-bit
                     | (2 << 21) // Dest Width 32-bit
                     | (1 << 26) // Source Increment: SI (recorrer array)
                     | (0 << 27) // Dest Increment: NO (siempre al mismo registro)
                     | (0 << 31); // Interrupt Disable

    GPDMA_Channel_CFG_Type GPDMACfg;
    GPDMACfg.ChannelNum      = 0; // Usamos canal 0
    GPDMACfg.SrcMemAddr      = (uint32_t)servo_buffer;
    GPDMACfg.DstMemAddr      = (uint32_t)&(LPC_TIM2->MR1);
    GPDMACfg.TransferSize    = BUFFER_SIZE;
    GPDMACfg.TransferType    = GPDMA_TRANSFERTYPE_M2P; // Memoria a Periférico
    GPDMACfg.SrcConn         = 0;
    // Trigger: Timer2 Match 0 (Sincronizado con el fin del periodo PWM)
    GPDMACfg.DstConn         = GPDMA_CONN_MAT2_0;
    GPDMACfg.DMALLI          = (uint32_t)&LLI_Play;

    GPDMA_Setup(&GPDMACfg);
}

// --- Configuración Botón ---
void ConfEINT1(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 11;
    PinCfg.Funcnum = 1;
    PINSEL_ConfigPin(&PinCfg);

    EXTI_InitTypeDef ExtiCfg;
    ExtiCfg.EXTI_Line = EXTI_EINT1;
    ExtiCfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    ExtiCfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_Config(&ExtiCfg);
    NVIC_EnableIRQ(EINT1_IRQn);
}

// --- IRQ Botón: Máquina de Estados ---
void EINT1_IRQHandler(void)
{
    for(volatile int i=0; i<1000000; i++); // Delay Anti-rebote
    EXTI_ClearEXTIFlag(EXTI_EINT1);

    systemMode = (systemMode + 1) % 3;

    // 1. Apagar DMA siempre al cambiar de estado (por seguridad)
    GPDMA_ChannelCmd(0, DISABLE);

    if (systemMode == 0)        // MANUAL
    {
        GPIO_ClearValue(0, 1<<LED_REC);
        NVIC_EnableIRQ(ADC_IRQn); 				// Habilitar interrupción ADC
    }
    else if (systemMode == 1)   // GRABAR - led prendido
    {
        GPIO_SetValue(0, 1<<LED_REC);
        write_index = 0;          // Reiniciar índice del buffer
        NVIC_EnableIRQ(ADC_IRQn); // Habilitar interrupción ADC (Aquí grabamos)
    }
    else                        // PLAY
    {
        GPIO_ClearValue(0, 1<<LED_REC);
        NVIC_DisableIRQ(ADC_IRQn); // Apagar ADC (para que no interfiera)

        SetupDMA_Play();           // Configurar y lanzar DMA
        GPDMA_ChannelCmd(0, ENABLE);
    }
}

// --- IRQ ADC: Lógica Principal de Control y Grabación ---
void ADC_IRQHandler(void)
{
    // Verificamos si hay dato listo
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        // 1. Lectura
        valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        // 2. Cálculo Matemático (Fórmula provista)
        if (valorADC1 >= 1948 && valorADC1 < 2147) {
            pulso = 1500;
        } else if (valorADC1 < 1948) {
            pulso = 1000 + (uint32_t)(((float)valorADC1 / 1947.0f) * 500.0f);
        } else {
            pulso = 1500 + (uint32_t)(((float)(valorADC1 - 2147) / 1948.0f) * 500.0f);
        }

        // 3. Actualizar Servo "en vivo" (Modo Manual y Rec)
        // Esto permite ver el movimiento mientras mueves el pote
        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);


        // --- NUEVO: SALIDA DAC EN VIVO ---
                uint32_t val_dac = pulso - 1000;
                DAC_UpdateValue(LPC_DAC, val_dac);
        // --------------------------------

        // 4. Si estamos en modo GRABACIÓN, guardamos en el array
        if (systemMode == 1)
        {
            servo_buffer[write_index] = pulso;
            write_index++;

            // Buffer Circular: Si llega al final, vuelve a 0
            if (write_index >= BUFFER_SIZE) {
                write_index = 0;
            }
        }
    }
}

// --- IRQ Timer2: Generación PWM ---
void TIMER2_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT)) // Inicio del ciclo (20ms)
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
        GPIO_SetValue(0, (1<<SERVO_PIN));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN));
    }
}

int main(void)
{
    GPDMA_Init();
    ConfGPIO();
    ConfADC();
    ConfDAC();    // <--- Agregar aquí
    ConfTIMER2();
    ConfEINT1();

    while(1)
    {
        __WFI();
    }
}
