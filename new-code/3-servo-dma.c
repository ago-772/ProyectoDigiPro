#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_exti.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>

/*******************************************************************************************
 *  DEFINICIONES IMPORTANTES (pines, ADC, servos, timers, botón DMA)
 *
 *  - Servos: P0.9, P0.8, P0.7   (3 servos)
 *  - ADC:   P0.26 (AD0.3), P1.30 (AD0.4), P1.31 (AD0.5)
 *  - UART:  TXD0 (P0.2), RXD0 (P0.3)
 *  - Botón DMA: P2.12 (EINT2)
 *******************************************************************************************/

// Mapeo servo
/*
static inline uint32_t mapServo(uint16_t val)
{
    if(val >=1948 && val<2147) return 1500;
    if(val <1948) return 1000 + (uint32_t)(((float)val /1947.0f)*500.0f);
    return 1500 + (uint32_t)(((float)(val-2147)/1948.0f)*500.0f);
}
*/
// Variables globales
volatile uint16_t valorADC1 = 0;
volatile uint16_t valorADC2 = 0;
volatile uint16_t valorADC3 = 0;

volatile uint32_t pulso = 1500;
volatile uint32_t pulso2 = 1500;
volatile uint32_t pulso3 = 1500;

// Pines servos
#define SERVO_PIN   9   // MR1
#define SERVO_PIN2  8   // MR2
#define SERVO_PIN3  7   // MR3

// ADC
#define ADC_CH   4
#define ADC_CH5  5
#define ADC_CH2  2

// Buffer movimiento
#define BUFFER_SIZE 32
#define N_SERVOS    3

volatile uint32_t counter = 0;

volatile uint32_t bufferMov[BUFFER_SIZE][N_SERVOS];
volatile uint32_t indiceGuardar = 0;

// Estados
typedef enum { MODO_RECORD = 0, MODO_PLAY = 1 } modo_t;
volatile modo_t modoOperacion = MODO_RECORD;

// DMA
#define DMA_CHANNEL_PLAY 0
volatile uint8_t dmaEstado = 0;						// dmaEstado: 0=idle, 1=TC, 2=ERR

// LLI (ahora globales para que no queden en la pila)
GPDMA_LLI_Type LLI1[BUFFER_SIZE];
GPDMA_LLI_Type LLI2[BUFFER_SIZE];
GPDMA_LLI_Type LLI3[BUFFER_SIZE];

//---------------------------------- UART ----------------------------------
void configGPIO_UART(void) {
    PINSEL_CFG_Type psel_conf;

    psel_conf.Portnum = 0;
    psel_conf.Pinmode = PINSEL_PINMODE_TRISTATE;
    psel_conf.OpenDrain = PINSEL_PINMODE_NORMAL;

    // TXD0 -> P0.2
    psel_conf.Pinnum = 2;
    psel_conf.Funcnum = 1;
    PINSEL_ConfigPin(&psel_conf);

    // RXD0 -> P0.3
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

// helper corto para enviar strings (opcional pero útil)
static inline void UART_SendText(const char *s)
{
    UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)s, strlen(s), BLOCKING);
}

//---------------------------------- GPIO SERVOS ----------------------------------
void ConfGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0; // GPIO
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

    // Servo 1
    PinCfg.Pinnum = SERVO_PIN;
    PINSEL_ConfigPin(&PinCfg);

    // Servo 2
    PinCfg.Pinnum = SERVO_PIN2;
    PINSEL_ConfigPin(&PinCfg);

    // Servo 3
    PinCfg.Pinnum = SERVO_PIN3;
    PINSEL_ConfigPin(&PinCfg);

    // Configurar como salida
    GPIO_SetDir(0, (1<<SERVO_PIN) | (1<<SERVO_PIN2) | (1<<SERVO_PIN3), 1);
}

//---------------------------------- ADC ----------------------------------
void ConfADC(void)
{
    PINSEL_CFG_Type PinCfg;

    // Potenciómetro 1 -> AD0.4 (P1.30)
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // Potenciómetro 2 -> AD0.5 (P1.31)
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);

    // Potenciómetro 3 -> AD0.2 (P0.25)
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 25;
    PinCfg.Funcnum = 1;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 1000);
    ADC_BurstCmd(LPC_ADC, ENABLE);

    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH5, ENABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CH2, ENABLE);

    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH5, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH2, ENABLE);

    ADC_StartCmd(LPC_ADC, ADC_START_CONTINUOUS);
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH, 1))
    {
        valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        if (valorADC1 >= 1948 && valorADC1 < 2147)
            pulso = 1500;
        else if (valorADC1 < 1948)
            pulso = 1000 + (uint32_t)(((float)valorADC1 / 1947.0f) * 500.0f);
        else
            pulso = 1500 + (uint32_t)(((float)(valorADC1 - 2147) / 1948.0f) * 500.0f);

        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }

    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH5, 1))
    {
        valorADC2 = ADC_ChannelGetData(LPC_ADC, ADC_CH5);

        if (valorADC2 >= 1948 && valorADC2 < 2147)
            pulso2 = 1500;
        else if (valorADC2 < 1948)
            pulso2 = 1000 + (uint32_t)(((float)valorADC2 / 1947.0f) * 500.0f);
        else
            pulso2 = 1500 + (uint32_t)(((float)(valorADC2 - 2147) / 1948.0f) * 500.0f);

        TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);
    }

    if (ADC_ChannelGetStatus(LPC_ADC, ADC_CH2, 1))
    {
        valorADC3 = ADC_ChannelGetData(LPC_ADC, ADC_CH2);

        if (valorADC3 >= 1948 && valorADC3 < 2147)
            pulso3 = 1500;
        else if (valorADC3 < 1948)
            pulso3 = 1000 + (uint32_t)(((float)valorADC3 / 1947.0f) * 500.0f);
        else
            pulso3 = 1500 + (uint32_t)(((float)(valorADC3 - 2147) / 1948.0f) * 500.0f);

        TIM_UpdateMatchValue(LPC_TIM2, 3, pulso3);
    }
}

//---------------------------------- TIMER0 (1 muestra/seg) ----------------------------------
void ConfTIMER0_Muestreo(void)
{
    TIM_TIMERCFG_Type c;
    c.PrescaleOption=TIM_PRESCALE_USVAL;
    c.PrescaleValue=1000;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &c);

    TIM_MATCHCFG_Type m;
    m.MatchChannel=0; m.IntOnMatch=ENABLE; m.ResetOnMatch=ENABLE;
    m.StopOnMatch=DISABLE; m.MatchValue=1000;

    TIM_ConfigMatch(LPC_TIM0,&m);
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

//en modo RECORD, guarda los valores actuales de MR1, MR2 y MR3

void TIMER0_IRQHandler(void)
{
    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

    if (modoOperacion == MODO_RECORD)
    {
        bufferMov[indiceGuardar][0] = LPC_TIM2->MR1;
        bufferMov[indiceGuardar][1] = LPC_TIM2->MR2;
        bufferMov[indiceGuardar][2] = LPC_TIM2->MR3;

        indiceGuardar = (indiceGuardar + 1) % BUFFER_SIZE;
    }
}

//---------------------------------- TIMER2 (PWM 3 servos) ----------------------------------
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

    TIM_MATCHCFG_Type match2;
    match2.MatchChannel = 2;
    match2.IntOnMatch = ENABLE;
    match2.ResetOnMatch = DISABLE;
    match2.StopOnMatch = DISABLE;
    match2.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match2.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match2);

    TIM_MATCHCFG_Type match3;
    match3.MatchChannel = 3;
    match3.IntOnMatch = ENABLE;
    match3.ResetOnMatch = DISABLE;
    match3.StopOnMatch = DISABLE;
    match3.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match3.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match3);

    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

void TIMER2_IRQHandler(void)
{
    // MATCH0 → reinicia ciclo PWM
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        counter++;

        if (counter >= 50)
        {
            char mensaje[128];

            sprintf(mensaje,
                    "Posicion de los servos : pulso=%u  pulso2=%u  pulso3=%u\r\n",
                    pulso, pulso2, pulso3);

            UART_Send((LPC_UART_TypeDef *)LPC_UART0, (uint8_t *)mensaje,strlen(mensaje),
                      BLOCKING);
            counter = 0;
        }

        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);

        GPIO_SetValue(0, (1<<SERVO_PIN));
        GPIO_SetValue(0, (1<<SERVO_PIN2));
        GPIO_SetValue(0, (1<<SERVO_PIN3));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN2));
    }

    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR3_INT))
    {
        TIM_ClearIntPending(LPC_TIM2, TIM_MR3_INT);
        GPIO_ClearValue(0, (1<<SERVO_PIN3));
    }
}

//---------------------------------- EINT2 (cambio RECORD/PLAY) ----------------------------------
void ConfEINT2_Button(void)
{
    PINSEL_CFG_Type p;
    p.Portnum=2;
    p.Pinnum=12;
    p.Funcnum=1;
    p.Pinmode=PINSEL_PINMODE_PULLUP;
    p.OpenDrain=0;
    PINSEL_ConfigPin(&p);

    EXTI_InitTypeDef e;
    e.EXTI_Line=EXTI_EINT2;
    e.EXTI_Mode=EXTI_MODE_EDGE_SENSITIVE;
    e.EXTI_polarity=EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE;

    EXTI_Config(&e);
    EXTI_ClearEXTIFlag(EXTI_EINT2);

    NVIC_EnableIRQ(EINT2_IRQn);
}

void DMA_Start(void); // forward declare

void EINT2_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT2);

    if (modoOperacion == MODO_RECORD)   //cambia a modo reproduccion
    {
        modoOperacion = MODO_PLAY;
        dmaEstado = 0;
        DMA_Start();   // arrancar DMA al entrar en PLAY
        return;
    }
    else								//cambia a modo grabacion
    {
        if (dmaEstado == 2) {
            // Hubo error en la reproducción
            UART_Send((LPC_UART_TypeDef *)LPC_UART0,
                      (uint8_t *)"DMA ERROR\r\n",
                      strlen("DMA ERROR\r\n"),
                      BLOCKING);
        } else if (dmaEstado == 1) {
            // transferencia completa (info opcional)
            UART_Send((LPC_UART_TypeDef *)LPC_UART0,
                      (uint8_t *)"DMA TC\r\n",
                      strlen("DMA TC\r\n"),
                      BLOCKING);
        }

        modoOperacion = MODO_RECORD;
        // deshabilitar los 3 canales si están activos
        GPDMA_ChannelCmd(0, DISABLE);
        GPDMA_ChannelCmd(1, DISABLE);
        GPDMA_ChannelCmd(2, DISABLE);
        dmaEstado = 0;      // limpiar
    }
}

//---------------------------------- DMA + LLI ----------------------------------

void DMA_Start(void)
{
    // Inicializar GPDMA (seguro hacerlo siempre antes)
    GPDMA_Init();

    // Construcción de LLI (circular) para cada servo
    for (int i=0; i<BUFFER_SIZE; i++)
    {
        // SERVO 1 → MR1
        LLI1[i].SrcAddr  = (uint32_t)&bufferMov[i][0];
        LLI1[i].DstAddr  = (uint32_t)&LPC_TIM2->MR1;
        LLI1[i].NextLLI  = (uint32_t)&LLI1[(i+1) % BUFFER_SIZE];
        LLI1[i].Control  = 1 | (GPDMA_WIDTH_WORD<<18) | (GPDMA_WIDTH_WORD<<21) | (1<<26) | (1<<27);

        // SERVO 2 → MR2
        LLI2[i].SrcAddr  = (uint32_t)&bufferMov[i][1];
        LLI2[i].DstAddr  = (uint32_t)&LPC_TIM2->MR2;
        LLI2[i].NextLLI  = (uint32_t)&LLI2[(i+1) % BUFFER_SIZE];
        LLI2[i].Control  = 1 | (GPDMA_WIDTH_WORD<<18) | (GPDMA_WIDTH_WORD<<21) | (1<<26) | (1<<27);

        // SERVO 3 → MR3
        LLI3[i].SrcAddr  = (uint32_t)&bufferMov[i][2];
        LLI3[i].DstAddr  = (uint32_t)&LPC_TIM2->MR3;
        LLI3[i].NextLLI  = (uint32_t)&LLI3[(i+1) % BUFFER_SIZE];
        LLI3[i].Control  = 1 | (GPDMA_WIDTH_WORD<<18) | (GPDMA_WIDTH_WORD<<21) | (1<<26) | (1<<27);
    }

    // Configuración canal 0 → MR1
    GPDMA_Channel_CFG_Type c1;
    c1.ChannelNum = 0;
    c1.SrcMemAddr = (uint32_t)&bufferMov[0][0];
    c1.DstMemAddr = (uint32_t)&LPC_TIM2->MR1;
    c1.TransferSize = 1;
    c1.TransferWidth = GPDMA_WIDTH_WORD;
    c1.TransferType = GPDMA_TRANSFERTYPE_M2M; // memoria a memoria (destino MRx)
    c1.SrcConn = 0;
    c1.DstConn = 0; // no peripheral connection
    c1.DMALLI = (uint32_t)&LLI1[0];
    GPDMA_Setup(&c1);
    GPDMA_ChannelCmd(0, ENABLE);

    // Canal 1 → MR2
    GPDMA_Channel_CFG_Type c2 = c1;
    c2.ChannelNum = 1;
    c2.SrcMemAddr = (uint32_t)&bufferMov[0][1];
    c2.DstMemAddr = (uint32_t)&LPC_TIM2->MR2;
    c2.DMALLI = (uint32_t)&LLI2[0];
    GPDMA_Setup(&c2);
    GPDMA_ChannelCmd(1, ENABLE);

    // Canal 2 → MR3
    GPDMA_Channel_CFG_Type c3 = c1;
    c3.ChannelNum = 2;
    c3.SrcMemAddr = (uint32_t)&bufferMov[0][2];
    c3.DstMemAddr = (uint32_t)&LPC_TIM2->MR3;
    c3.DMALLI = (uint32_t)&LLI3[0];
    GPDMA_Setup(&c3);
    GPDMA_ChannelCmd(2, ENABLE);

    // Habilitar IRQ del GPDMA
    NVIC_EnableIRQ(DMA_IRQn);
}

void DMA_IRQHandler(void)
{
    // Revisar errores y completados por canal (0,1,2)
    // Si cualquier canal muestra INTERR -> setear dmaEstado = 2 (error)
    for (int ch = 0; ch <= 2; ch++)
    {
        if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, ch))
        {
            GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, ch);
            dmaEstado = 2; // error
        }

        if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, ch))
        {
            GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, ch);
            dmaEstado = 1; // transferencia completa (al menos un TC)
        }
    }
}

//---------------------------------- MAIN ----------------------------------
int main(void)
{
    ConfGPIO();
    ConfADC();
    ConfTIMER2();
    configGPIO_UART();
    configUART();
    ConfTIMER0_Muestreo();
    ConfEINT2_Button();

    GPDMA_Init();

    while(1){

    }

    return 0;
}
