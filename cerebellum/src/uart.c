/**
 * @file src/uart.c
 * @brief Basic UART IO driver
 * @author WebConn
 * @date 5 Jan 2013
 */

#include <cerebellum/uart.h>

#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#ifdef __GNUC__
    #define PUTCHAR_PROTORYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

void uart_init(uint32_t uart, uint32_t baudrate)
{
    USART_InitTypeDef usart = {
        .USART_BaudRate = baudrate,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Tx | USART_Mode_Rx
    };

    GPIO_InitTypeDef rx = {
        .GPIO_Mode = GPIO_Mode_IN_FLOATING
    };

    GPIO_InitTypeDef tx = {
        .GPIO_Mode = GPIO_Mode_AF_PP,
        .GPIO_Speed = GPIO_Speed_50MHz
    };

    if(uart == 1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        tx.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOA, &tx);
        rx.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOA, &rx);

        USART_Init(USART1, &usart);
        USART_Cmd(USART1, ENABLE);
    }
    else if(uart == 2)
    {
        USART_Init(USART2, &usart);
        USART_Cmd(USART2, ENABLE);
    }
    else if(uart == 3)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

        tx.GPIO_Pin = GPIO_Pin_10;
        rx.GPIO_Pin = GPIO_Pin_11;

        GPIO_Init(GPIOB, &tx);
        GPIO_Init(GPIOB, &tx);

        USART_Init(USART3, &usart);
        USART_Cmd(USART3, ENABLE);
    }
}

int uart_send(int uart, int ch)
{
    if(uart == 1)
    {
        USART_SendData(USART1, (uint8_t) ch);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);;;
    }
    else if(uart == 2)
    {
        USART_SendData(USART2, (uint8_t) ch);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);;;
    }
    else if(uart == 3)
    {
        USART_SendData(USART3, (uint8_t) ch);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);;;
    }

    return ch;
}

// Don't touch this function! :) It just works
int _write(int file, char * ptr, int len)
{
    uart_send(CONFIG_UART_STDOUT, *(ptr-1));
    if(len == 1) uart_send(3, *ptr);
    return 1;
}
