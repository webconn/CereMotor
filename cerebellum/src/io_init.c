#include <arch/antares.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <stm32f10x.h>
#include <lib/tasks.h>

/*
void SysTick_Handler(void)
{
    tmgr_tick();
}
*/

GPIO_InitTypeDef LED_Init = {
    .GPIO_Pin = GPIO_Pin_5,
    .GPIO_Speed = GPIO_Speed_50MHz,
    .GPIO_Mode = GPIO_Mode_Out_PP
};

USART_InitTypeDef CereSat_UART = {
    .USART_BaudRate = CONFIG_SAT_UART_BAUDRATE,
    .USART_WordLength = USART_WordLength_8b,
    .USART_StopBits = USART_StopBits_1,
    .USART_Parity = USART_Parity_Even,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
    .USART_Mode = USART_Mode_Tx | USART_Mode_Rx
};

inline void UART3_Init(void)
{
    // To init USART, we need to run clock
    // USART3 is on the APB1 domain
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Now configuring UART itself
    // Preferences: baudrate is set from Antares, even parity, 8 bit,
    // one stop bit, hardware flow control disabled, full duplex
    USART_Init(USART3, &CereSat_UART);    

    // Configuring I/O ports
    // USART3 pins: PB11 as Rx, PB10 as Tx
    GPIO_InitTypeDef USART3_Rx_GPIO = {
        .GPIO_Pin = GPIO_Pin_11,
        .GPIO_Mode = GPIO_Mode_IN_FLOATING
    };

    GPIO_InitTypeDef USART3_Tx_GPIO = {
        .GPIO_Pin = GPIO_Pin_10,
        .GPIO_Mode = GPIO_Mode_AF_PP,
        .GPIO_Speed = GPIO_Speed_10MHz
    };

    GPIO_Init(GPIOB, &USART3_Rx_GPIO);
    GPIO_Init(GPIOB, &USART3_Tx_GPIO);
}


inline void UART2_Init(void)
{
    // To init USART, we need to run clock
    // USART3 is on the APB1 domain
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Now configuring UART itself
    // Preferences: baudrate is set from Antares, even parity, 8 bit,
    // one stop bit, hardware flow control disabled, full duplex
    USART_Init(USART2, &CereSat_UART);    

    // Configuring I/O ports
    // USART3 pins: PB11 as Rx, PB10 as Tx
    GPIO_InitTypeDef USART2_Rx_GPIO = {
        .GPIO_Pin = GPIO_Pin_3,
        .GPIO_Mode = GPIO_Mode_IN_FLOATING
    };

    GPIO_InitTypeDef USART2_Tx_GPIO = {
        .GPIO_Pin = GPIO_Pin_2,
        .GPIO_Mode = GPIO_Mode_AF_PP,
        .GPIO_Speed = GPIO_Speed_10MHz
    };

    GPIO_Init(GPIOA, &USART2_Rx_GPIO);
    GPIO_Init(GPIOA, &USART2_Tx_GPIO);
}

int main(void)
{
    // 1. Init UART interfaces with CereSats
    
    UART3_Init(); // Right channel
    UART2_Init(); // Left channel




    return 0;
}

void mdelay(int n)
{
    sleep_ticks(n);
}
