#ifndef CEREBELLUM_UART_H
#define CEREBELLUM_UART_H

#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>


void uart_init(uint32_t uart, uint32_t baudrate);
int uart_send(int uart, int ch);
uint8_t uart_read(int uart);

#endif
