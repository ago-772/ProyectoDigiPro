/* --------------------------------------------------------------
 * DAC BLOQUE 1 — Pulso del servo convertido a voltaje (0–3.3V)
 *
 * Qué hace:
 *   pulso = 1000–2000 us  → DAC = 0–3.3V proporcional
 *   Se ve una señal analógica suave en el osciloscopio.
 * -------------------------------------------------------------- */

#include "lpc17xx_dac.h"
#include "lpc17xx_pinsel.h"

volatile uint16_t dacValue = 0;

/* Configurar DAC en P0.26 */
void ConfDAC_PulseScale(void)
{
    PINSEL_CFG_Type p;
    p.Portnum = 0;
    p.Pinnum  = 26;
    p.Funcnum = 2;   // DAC
    p.Pinmode = PINSEL_PINMODE_TRISTATE;
    p.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&p);

    DAC_Init(LPC_DAC);
}

/* Convertir pulso (1000–2000us) a DAC (0–1023 → 0–3.3V) */
void DAC_Update_FromPulse(uint32_t pulse_us)
{
    // Escala lineal → 1000us = 0V, 2000us = 3.3V
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;

    uint32_t x = pulse_us - 1000;   // rango: 0–1000
    dacValue = (x * 1023) / 1000;   // escala a 0–1023

    DAC_UpdateValue(LPC_DAC, dacValue);
}
