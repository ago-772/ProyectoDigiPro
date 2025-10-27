#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"

// --- Prototipos ---
void configServoPWM(void);
void delay_ms(uint32_t ms);

// --------------------------------------------------------------------------
// FUNCIÓN PRINCIPAL
// --------------------------------------------------------------------------
int main(void) {
    // Configurar el Timer para generar la señal PWM del servo
    configServoPWM();

    while(1) {
        // --- Ciclo de Prueba: Mover el servo a 0°, 90° y 180° ---

        // 1. Mover a 0 grados (pulso de 1 ms = 1000 µs)
        TIM_UpdateMatchValue(LPC_TIM0, 1, 1000);
        delay_ms(2000); // Esperar 2 segundos para que el servo llegue

        // 2. Mover a 90 grados (pulso de 1.5 ms = 1500 µs)
        TIM_UpdateMatchValue(LPC_TIM0, 1, 1500);
        delay_ms(2000); // Esperar 2 segundos

        // 3. Mover a 180 grados (pulso de 2 ms = 2000 µs)
        TIM_UpdateMatchValue(LPC_TIM0, 1, 2000);
        delay_ms(2000); // Esperar 2 segundos
    }
    return 0;
}

// --------------------------------------------------------------------------
// CONFIGURACIÓN DEL TIMER PARA LA SEÑAL PWM DEL SERVO
// --------------------------------------------------------------------------
void configServoPWM(void) {
    TIM_TIMERCFG_Type timerCfg;
    TIM_MATCHCFG_Type matchCfg;
    PINSEL_CFG_Type pinPWM;

    // --- Configuración del Timer ---
    
    // Configurar el Timer para que el tick sea de 1 microsegundo (µs)
    timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timerCfg.PrescaleValue = 1;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

    // Configurar MR0 para el período de la señal: 20ms (frecuencia de 50Hz)
    matchCfg.MatchChannel = 0;
    matchCfg.MatchValue = 20000;      // 20000 µs = 20 ms
    matchCfg.ResetOnMatch = ENABLE;   // Resetear el contador para repetir el ciclo
    matchCfg.IntOnMatch = DISABLE;    // No necesitamos interrupción
    matchCfg.StopOnMatch = DISABLE;
    matchCfg.ExtMatchAction = TIM_EXTMATCH_NOTHING;
    TIM_ConfigMatch(LPC_TIM0, &matchCfg);

    // Configurar MR1 para el ancho del pulso inicial (ej. 90 grados = 1.5ms)
    matchCfg.MatchChannel = 1;
    matchCfg.MatchValue = 1500;       // 1500 µs = 1.5 ms
    matchCfg.ResetOnMatch = DISABLE;
    matchCfg.IntOnMatch = DISABLE;    // No necesitamos interrupción
    TIM_ConfigMatch(LPC_TIM0, &matchCfg);

    // --- Configuración del Pin de Salida PWM ---
    
    // El canal 1 del PWM (controlado por MR1) usa el pin MAT0.1