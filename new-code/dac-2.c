/* --------------------------------------------------------------
 * DAC BLOQUE 2 — Representación analógica del PWM del servo
 *
 * Qué hace:
 *   - MATCH0 → inicio pulso servo 1 → DAC = 3.3V
 *   - MATCH1 → fin pulso servo 1   → DAC = 0V
 *
 * Ideal para osciloscopio (forma del PWM real).
 * -------------------------------------------------------------- */

#include "lpc17xx_dac.h"
#include "lpc17xx_pinsel.h"

/* Configurar DAC en P0.26 */
void ConfDAC_PWM(void)
{
    PINSEL_CFG_Type p;
    p.Portnum = 0;
    p.Pinnum  = 26;
    p.Funcnum = 2;     // DAC
    p.Pinmode = PINSEL_PINMODE_TRISTATE;
    p.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&p);

    DAC_Init(LPC_DAC);
}

/* Para agregar dentro del TIMER2_IRQHandler */
void DAC_PWM_Handler(void)
{
    // Inicio de PWM → pulso en ALTO
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT))
    {
        DAC_UpdateValue(LPC_DAC, 1023); // 3.3V
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
    }

    // Fin del pulso del servo 1
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR1_INT))
    {
        DAC_UpdateValue(LPC_DAC, 0);    // 0V
        TIM_ClearIntPending(LPC_TIM2, TIM_MR1_INT);
    }
}
