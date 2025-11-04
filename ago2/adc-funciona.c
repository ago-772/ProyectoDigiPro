#include "LPC17xx.h"

#include "lpc17xx_timer.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"

// TODO: insert other definitions and declarations here

#define	OUTPUT	    (uint8_t) 1
#define INPUT	    (uint8_t) 0

#define PIN_22		((uint32_t)(1<<22))
#define PORT_ZERO	(uint8_t)	0
#define PORT_ONE	(uint8_t)	1
#define PORT_TWO	(uint8_t)	2
#define PORT_THREE	(uint8_t)	3
#define _ADC_INT		ADC_ADINTEN2
#define _ADC_CHANNEL		ADC_CHANNEL_2

__IO uint32_t adc_value;

void ADC_IRQHandler(void);
void configPin(void);
void configADC(void);

void config_GPIO(void);
void config_timer(void);

void TIMER0_IRQHandler(void);

int main(void) {

	config_GPIO();
	config_timer();
	GPIO_SetValue(PORT_ZERO,PIN_22);
	
	uint32_t tmp;

	configPin();
	configADC();

	while(1) {
		
		ADC_StartCmd(LPC_ADC,ADC_START_NOW);

		for(tmp = 0; tmp < 1000000; tmp++);
		
	}
    return 0 ;
}

void config_GPIO(){
	PINSEL_CFG_Type pin_configuration;

	pin_configuration.Portnum 	=	PINSEL_PORT_0;
	pin_configuration.Pinnum	=	PINSEL_PIN_22;
	pin_configuration.Pinmode	=	PINSEL_PINMODE_PULLUP;
	pin_configuration.Funcnum	= 	PINSEL_FUNC_0;
	pin_configuration.OpenDrain	=	0;

	PINSEL_ConfigPin(&pin_configuration);

	GPIO_SetDir( PORT_ZERO , PIN_22 , OUTPUT );

	return;
}

void config_timer(){
	TIM_TIMERCFG_Type	struct_config;
	TIM_MATCHCFG_Type	struct_match;

	struct_config.PrescaleOption	=	TIM_PRESCALE_USVAL;
	struct_config.PrescaleValue		=	1;

	struct_match.MatchChannel		=	0;
	struct_match.IntOnMatch			=	ENABLE;
	struct_match.ResetOnMatch		=	ENABLE;
	struct_match.StopOnMatch		=	DISABLE;
	struct_match.ExtMatchOutputType	=	TIM_EXTMATCH_NOTHING;
	struct_match.MatchValue			=	1000;

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &struct_config);
	TIM_ConfigMatch(LPC_TIM0, &struct_match);

	NVIC_EnableIRQ(TIMER0_IRQn);
	
	TIM_Cmd(LPC_TIM0, ENABLE);

	return;
}

void configPin(){
	PINSEL_CFG_Type PinCfg;
	/*
	 * Init ADC pin connect
	 * AD0.2 on P0.25
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 25;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
return;
}

void configADC(){

	/* Configuration for ADC:
	 *  select: ADC channel 2
	 *  ADC conversion rate = 200KHz
	 */
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,_ADC_INT,ENABLE);
	ADC_ChannelCmd(LPC_ADC,_ADC_CHANNEL,ENABLE);

	NVIC_SetPriority(ADC_IRQn, (9));
	/* Enable ADC in NVIC */
	NVIC_EnableIRQ(ADC_IRQn);
	return;
}

void ADC_IRQHandler(void)
{
	adc_value = 0;
	if (ADC_ChannelGetStatus(LPC_ADC,_ADC_CHANNEL,ADC_DATA_DONE))
	{
		adc_value =  ADC_ChannelGetData(LPC_ADC,_ADC_CHANNEL);
	}
	
	uint16_t pulse_width_us;
	
	pulse_width_us = 1000 + ((uint32_t)adc_value * 1000 / 4095);
	
	TIM_UpdateMatchValue(LPC_TIM0, 0, pulse_width_us);
	
	ADC_ClearIntPending(LPC_ADC, ADC_ADINTEN2); 
	
}

void TIMER0_IRQHandler(void){
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

	if (GPIO_ReadValue(PORT_ZERO)&PIN_22) {
		GPIO_ClearValue(PORT_ZERO, PIN_22);
	} else {
		GPIO_SetValue(PORT_ZERO,PIN_22);
	}

	return;
}