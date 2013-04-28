#ifndef CEREBELLUM_UARTGRAB_H
#define CEREBELLUM_UARTGRAB_H

#include <stm32f10x.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_rcc.h>

#include <cerebellum/uart.h>

void uartgrab_init(uint8_t uart, uint32_t baudrate, uint8_t length);
uint8_t * uartgrab_get(uint8_t uart);

#endif
