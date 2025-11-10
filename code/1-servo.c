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
                
        TIM_UpdateMatchValue(LPC_TIM2, 1, pulso);
    }