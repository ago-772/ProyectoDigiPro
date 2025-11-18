void SetupDMA_Record(void)
{
    GPDMA_Channel_CFG_Type GPDMACfg;

    GPDMACfg.ChannelNum      = 0;
    GPDMACfg.TransferType    = GPDMA_TRANSFERTYPE_M2M;   // RAM → RAM
    GPDMACfg.SrcMemAddr      = (uint32_t)&pulso_actual;  // variable RAM
    GPDMACfg.DstMemAddr      = (uint32_t)servo_buffer;   // buffer RAM
    GPDMACfg.TransferSize    = BUFFER_SIZE;
    GPDMACfg.TransferWidth   = 2;                        // word 32 bits
    GPDMACfg.SrcConn         = 0;                        // NO periféricos
    GPDMACfg.DstConn         = 0;                        // NO periféricos
    GPDMACfg.DMALLI          = 0;

    GPDMA_Setup(&GPDMACfg);
}