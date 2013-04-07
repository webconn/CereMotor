#include <cerebellum/odetect.h>

volatile uint8_t odetect_data = 0;

void odetect_init(void)
{
    // Setup UART3 to work with DMA
    // UART3 Rx trigger is on DMA1 Channel 3
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitTypeDef dma = {
        .DMA_Mode = DMA_Mode_Circular,
        .DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR),
        .DMA_MemoryBaseAddr = (uint32_t) & odetect_data,
        .DMA_DIR = DMA_DIR_PeripheralSRC,
        .DMA_BufferSize = 1,
        .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
        .DMA_MemoryInc = DMA_MemoryInc_Enable,
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
        .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
        .DMA_Priority = DMA_Priority_High,
        .DMA_M2M = DMA_M2M_Disable
    };
    DMA_Init(DMA1_Channel3, &dma);

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}

int odetect_getDirection(int direction)
{
    return (odetect_data & direction);
}

uint8_t odetect_getRaw(void)
{
    return odetect_data;
}
