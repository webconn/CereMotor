#include <cerebellum/uartgrab.h>

uint8_t bytes1[16];
uint8_t bytes2[16];
uint8_t bytes3[16];

uint8_t len1 = 0;
uint8_t len2 = 0;
uint8_t len3 = 0;

void uartgrab_init(uint8_t uart, uint32_t baudrate, uint8_t length)
{
    // Init UART
    uart_init(uart, baudrate);

    // Init DMA
    if(uart == 1)
    {
        len1 = length;
        // Init DMA1 channel 5
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        DMA_InitTypeDef dma = {
            .DMA_Mode = DMA_Mode_Circular,
            .DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR),
            .DMA_MemoryBaseAddr = (uint32_t) bytes1,
            .DMA_DIR = DMA_DIR_PeripheralSRC,
            .DMA_BufferSize = length,
            .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
            .DMA_MemoryInc = DMA_MemoryInc_Enable,
            .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
            .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
            .DMA_Priority = DMA_Priority_High,
            .DMA_M2M = DMA_M2M_Disable
        };
        DMA_Init(DMA1_Channel5, &dma);

        USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
    else if(uart == 2)
    {
        len2 = length;
        // Init DMA1 channel 6
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        DMA_InitTypeDef dma = {
            .DMA_Mode = DMA_Mode_Circular,
            .DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR),
            .DMA_MemoryBaseAddr = (uint32_t) bytes2,
            .DMA_DIR = DMA_DIR_PeripheralSRC,
            .DMA_BufferSize = length,
            .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
            .DMA_MemoryInc = DMA_MemoryInc_Enable,
            .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
            .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
            .DMA_Priority = DMA_Priority_High,
            .DMA_M2M = DMA_M2M_Disable
        };
        DMA_Init(DMA1_Channel6, &dma);

        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA1_Channel6, ENABLE);
    }
    else if(uart == 3)
    {
        len3 = length;
        // Init DMA1 channel 3
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        DMA_InitTypeDef dma = {
            .DMA_Mode = DMA_Mode_Circular,
            .DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR),
            .DMA_MemoryBaseAddr = (uint32_t) bytes3,
            .DMA_DIR = DMA_DIR_PeripheralSRC,
            .DMA_BufferSize = length,
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
}

uint8_t * uartgrab_get(uint8_t uart)
{
    if(uart == 1)
        return bytes1;
    else if(uart == 2)
        return bytes2;
    else if(uart == 3)
        return bytes3;

    return (uint8_t *) 0;
}

