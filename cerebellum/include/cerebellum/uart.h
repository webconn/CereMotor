#ifndef CEREBELLUM_UART_H
#define CEREBELLUM_UART_H

#include <stm32f10x.h>

void uart_init(uint32_t uart, uint32_t baudrate);
int uart_send(int uart, int ch);

#endif
