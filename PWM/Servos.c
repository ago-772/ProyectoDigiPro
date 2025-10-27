/*	Drivers
 *  Created on: 14 october 2025
 *      Author: jnach
 *      
 *      
 */
#include "lpc17xx_gpio.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_uart.h"

void configPotenciometro();

uint32_t adc_value;
uint32_t valor_match_pwm;

int main(){
    configPotenciometro();  
    configPWM_Servo();

    while(1) {
    }

    return 0;
}

void configPotenciometro(void) {    
    PINSEL_CFG_Type PinCfg;

    PinCfg.Funcnum = 1;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 23;
    PinCfg.Pinmode = 0; 
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 100000);

    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
    ADC_IntConfg(LPC_ADC , ADC_ADINTEN0, ENABLE);

    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void) {
    if (ADC_GetStatus(LPC_ADC, ADC_ADINTEN0, ADC_FLAG_DONE)) {
        
        adc_valor = ADC_GetData(LPC_ADC, ADC_CHANNEL_0);
        
        valor_match_pwm = 1000 + (uint32_t)(((float)adc_valor / 4095.0f) * 1000.0f);
        
        TIM_UpdateMatchValue(LPC_TIM0, 1, valor_match_pwm);
    }
}

void configTimer(void){
    GPIO_SetDir(1 , (1 << 28), 0);
    PINSEL_CFG_Type pinty = {PINSEL_PORT_1, PINSEL_PIN_28, PINSEL_FUN_2, PINSEL_PINMODE_TRISTATE , PINSEL_PINMODE_NORMAL};
    PINSEL_ConfigPin(&pinty);

    GPIO_SetDir(1 , (1 << 29), 0);
    PINSEL_CFG_Type pinty1 = {PINSEL_PORT_1, PINSEL_PIN_29, PINSEL_FUN_2, PINSEL_PINMODE_TRISTATE , PINSEL_PINMODE_NORMAL};
    PINSEL_ConfigPin(&pinty1);
    

    TIM_TIMERCFG_Type timty = {TIM_PRESCALE_USVAL , 1};
    TIM_MATCHCFG_Type matchty = {0 , ENABLE , DISABLE , ENABLE , TIM_EXTMATCH_NOTHING , 20000};

    TIM_MATCHCFG_Type matchty1 = {1 , ENABLE , DISABLE , DISABLE , TIM_EXTMATCH_NOTHING , 1};

    TIM_Init(LPC_TIM0 , TIM_TIMER , &timty);
    
    TIM_ConfigMatch(LPC_TIM0 , &matchty);
    TIM_ConfigMatch(LPC_TIM0 , &matchty1);

    TIM_Cmd(LPC_TIM0, ENABLE);

    // Habilitar la salida PWM en el pin correspondiente (ej. P1.26 para MAT0.0)
    LPC_TIM0->PCR |= (1 << 9);
    GPIO_SetDir(1 , (1 << 26), 1);
    PINSEL_CFG_Type pwm = {PINSEL_PORT_1, PINSEL_PIN_26, PINSEL_FUN_2, PINSEL_PINMODE_TRISTATE , PINSEL_PINMODE_NORMAL};
    PINSEL_ConfigPin(&pwm);

    NVIC_EnableIRQ(TIMER0_IRQn);
}

void TIMER0_IRQHandler(void) {
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT) == SET) {

        ADC_StartCmd(LPC_ADC, ADC_START_NOW);

        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
    }
}