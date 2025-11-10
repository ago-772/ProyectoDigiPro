/*
 * Probando facilmente 2 protenciometro con leds
 *
 *
 */

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_nvic.h"

/*
 * potenciometro por el pin - P0.9
 * mediante el ADC (canal AD0.4) y PWM por software con Timer2. - P1.30
 */

#define SERVO_PIN 9        // P0.9
#define SERVO_PIN2 8        // P0.8
#define ADC_CH    4        // AD0.4 (P1.30)
#define ADC_CH5    5        // AD0.5 (P1.31)
#define SERVO_MIN_US 1000            // 1.0 ms  -> ángulo mínimo
#define SERVO_MAX_US 2000            // 2.0 ms  -> ángulo máximo
#define BUTTON 6            //P0.6
#define BUFFER_SIZE 5000 	// 10 segundos / 2 ms = 5000 muestras

// --- Buffers de grabación ---
// Guardamos el *ancho de pulso* (1000-2000), no el valor ADC (0-4095)
volatile uint16_t buffer_servo1[BUFFER_SIZE];
volatile uint16_t buffer_servo2[BUFFER_SIZE];

volatile uint32_t write_index = 0; // Dónde estamos escribiendo
volatile uint32_t read_index = 0;  // Dónde estamos leyendo

volatile uint16_t valorADC1 = 0;
volatile uint16_t valorADC2 = 0;

volatile uint32_t pulso=0;
volatile uint32_t pulso2=0;

volatile uint32_t contador = 0 ;

// --- Máquina de Estados ---
typedef enum {
	STATE_RECORDING, // Estamos grabando
	STATE_PLAYING	 // Estamos reproduciendo
} SystemState;

volatile SystemState currentState = STATE_RECORDING;

void ConfGPIO(void)
{
	//configuracion pin P0.9
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Funcnum = 0;
    PinCfg.Pinnum = SERVO_PIN;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;    

    PINSEL_ConfigPin(&PinCfg);

    // CONFIGURO EL BOTON
    PinCfg.Pinnum = BUTTON;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0 , (1 << 6) , 0);
    GPIO_IntCmd(0 , (1 << 6) , 1);         //botton activo por bajo

    NVIC_EnableIRQ(EINT3_IRQn);

    GPIO_SetDir(0,1<<22,1);
    GPIO_SetDir(3,1<<25,1);
    GPIO_SetDir(3,1<<26,1);
     

    PinCfg.Pinnum = SERVO_PIN2;
    PINSEL_ConfigPin(&PinCfg);

    GPIO_SetDir(0, (1<<SERVO_PIN), 1);					//configurar como salida
    GPIO_SetDir(0, (1<<SERVO_PIN2), 1);
}

void EINT3_IRQHandler()
{
	if (GPIO_GetIntStatus(0, (1 << 6), 1)) 
	{
		GPIO_ClearInt(LPC_GPIOINT, 0, (1 << BUTTON)); 

		if (currentState == STATE_RECORDING)
		{
			// Cambia a modo REPRODUCCIÓN
			currentState = STATE_PLAYING;

			// Calculamos dónde empezar a leer:
			// El dato más viejo está 1 posición "adelante"
			// del que acabamos de escribir (circular).
			read_index = (write_index + 1) % 5000;
		}else // currentState == STATE_PLAYING
		{
			// --- MODO REPRODUCCIÓN -> PASAR A GRABAR (CANCELAR) ---
			currentState = STATE_RECORDING;
			
			// IMPORTANTE: Para evitar un "salto" brusco, 
			// le decimos a los servos que vuelvan a la posición
			// ACTUAL del joystick, devolviéndole el control.
			TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
			TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);
		}
    }
}

void ConfADC(void)
{
    // Pin P1.30 -> AD0.4
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 30;
    PinCfg.Funcnum = 3;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.OpenDrain = 0;
    PINSEL_ConfigPin(&PinCfg);

    // Pin P1.31 -> AD0.5
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);


    ADC_Init(LPC_ADC, 1000);       // 200 kHz
    ADC_BurstCmd(LPC_ADC, ENABLE);   // Burst Mode
    ADC_ChannelCmd(LPC_ADC, ADC_CH, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH, ENABLE);

    ADC_ChannelCmd(LPC_ADC, ADC_CH5, ENABLE);
    ADC_IntConfig(LPC_ADC, ADC_CH5, ENABLE);

    ADC_StartCmd(LPC_ADC,ADC_START_CONTINUOUS);

    NVIC_EnableIRQ(ADC_IRQn);


}


// INTERRUPCIÓN ADC
void ADC_IRQHandler(void) {
    // Lee valor del canal 4
    if(ADC_ChannelGetStatus(LPC_ADC, ADC_CH , 1 )){

        valorADC1 = ADC_ChannelGetData(LPC_ADC, ADC_CH);

        if(valorADC1>=1948 && valorADC1 < 2147){
            pulso = 1500;
                }else if(valorADC1>=0 && valorADC1< 1948){
                    pulso = 1000 + ((uint32_t)valorADC1 * 500 / 1947);
                }else if(valorADC1>=2147 && valorADC1<4096){
                    pulso = 1500 + ((uint32_t)valorADC1 *500 / 1947);
                }

    }

if(ADC_ChannelGetStatus(LPC_ADC, ADC_CH5 , 1)){
// Lee valor del canal 5  (ejey)
    valorADC2 = ADC_ChannelGetData(LPC_ADC, ADC_CH5);

    // Mapea 0–4095 → 1000–2000 µs

    if(valorADC2>=1948 && valorADC2 < 2147){
        pulso2 = 1500;
            }else if(valorADC2>=0 && valorADC2< 1948){
                pulso2 = 1000 + ((uint32_t)valorADC2 * 500 / 1947);
            }else if(valorADC2>=2147 && valorADC2<4096){
                pulso2 = 1500 + ((uint32_t)valorADC2 * 500 / 1947);
            }
        // Actualiza el valor del MATCH2 (ancho de pulso)
    }

    if (currentState == STATE_RECORDING)
	{
		TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
		TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);
	}
}


// CONFIGURACIÓN TIMER2
void ConfTIMER2(void)
{
    TIM_TIMERCFG_Type timerCfg;
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;

    TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timerCfg);

    // MR0 = periodo (20 ms)
    TIM_MATCHCFG_Type match0;
    match0.MatchChannel = 0;
    match0.IntOnMatch = ENABLE;							//en la interrupcion pone el pin en ALTO
    match0.ResetOnMatch = ENABLE;
    match0.StopOnMatch = DISABLE;
    match0.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match0.MatchValue = 20000;
    TIM_ConfigMatch(LPC_TIM2, &match0);

    // MR1 = duty inicial
    TIM_MATCHCFG_Type match1;
    match1.MatchChannel = 1;
    match1.IntOnMatch = ENABLE;							//en la interrupcion pone el pin en BAJO
    match1.ResetOnMatch = DISABLE;
    match1.StopOnMatch = DISABLE;
    match1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match1.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match1);

        // MR2 = duty inicial
    TIM_MATCHCFG_Type match2;
    match2.MatchChannel = 2;
    match2.IntOnMatch = ENABLE;							//en la interrupcion pone el pin en BAJO
    match2.ResetOnMatch = DISABLE;
    match2.StopOnMatch = DISABLE;
    match2.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match2.MatchValue = 1500;
    TIM_ConfigMatch(LPC_TIM2, &match2);


        // MR3 = Duty inicial
    match2.MatchChannel = 3;						//Reutilizo el match2 para crear el MATCH3
    match2.ResetOnMatch = ENABLE;
    match2.MatchValue = 2000;                   //Int cada 2ms
    TIM_ConfigMatch(LPC_TIM2, &match2);


    NVIC_EnableIRQ(TIMER2_IRQn);
    TIM_Cmd(LPC_TIM2, ENABLE);
}

// INTERRUPCIÓN TIMER2
void TIMER2_IRQHandler(void)
{
	//interrupion por MATCH0 (Inicio de período PWM)
	if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT)) {
		TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT); 	//limpiar bandera
		GPIO_SetValue(0, (1<<SERVO_PIN)); 				//poner en ALTO el pin P0.9
		GPIO_SetValue(0, (1<<SERVO_PIN2));
	}

	//interrupion por MATCH1 (Fin pulso Servo 1)
	if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT)) {
		TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT); 	//limpiar bandera
		GPIO_ClearValue(0, (1<<SERVO_PIN)); 			//poner en BAJO el pin P0.9
	}

	//interrupion por MATCH2 (Fin pulso Servo 2)
	if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT)) {
		TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT); 	//limpiar bandera
		GPIO_ClearValue(0, (1<<SERVO_PIN2)); 			//poner en BAJO el pin P0.8
	}

	// --- LÓGICA DE GRABACIÓN / REPRODUCCIÓN (Cada 2ms) ---
	if (TIM_GetIntStatus(LPC_TIM2, TIM_MR3_INT)) {
		TIM_ClearIntPending(LPC_TIM2, TIM_MR3_INT); 

		if (currentState == STATE_RECORDING)
		{
			// --- MODO GRABACIÓN ---
			// Usamos 'write_index' (no 'contador')
			// Guardamos el *pulso* (1000-2000), no el valor ADC
			// Guardamos en 'buffer_servo1' y 'buffer_servo2'
			buffer_servo1[write_index] = (uint16_t)pulso;
			buffer_servo2[write_index] = (uint16_t)pulso2;
			
			// Avanza el índice de escritura (circular)
			write_index = (write_index + 1) % BUFFER_SIZE;
		}
		else // currentState == STATE_PLAYING
		{
			// --- MODO REPRODUCCIÓN ---
			// Leemos el valor guardado
			uint16_t s1_play = buffer_servo1[read_index];
			uint16_t s2_play = buffer_servo2[read_index];

			// Actualizamos los servos con el valor del buffer
			TIM_UpdateMatchValue(LPC_TIM2, 1, s1_play);
			TIM_UpdateMatchValue(LPC_TIM2, 2, s2_play);

			// Avanza el índice de lectura (circular)
			read_index = (read_index + 1) % BUFFER_SIZE;

			// Si dimos la vuelta completa (read_index alcanzó a write_index),
			// significa que terminamos de reproducir.
			if (read_index == write_index)
			{
				currentState = STATE_RECORDING; // Volver a grabar
				
				// Transición suave: devolvemos control al joystick
				TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
				TIM_UpdateMatchValue(LPC_TIM2, 2, pulso2);
			}
		}
	}
}

int main(){
    ConfGPIO();
    ConfADC();
    ConfTIMER2();

    while(1){

    }

}