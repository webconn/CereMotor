#ifndef CEREBELLUM_ODETECT_H
#define CEREBELLUM_ODETECT_H

#include <stm32f10x.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_rcc.h>

#define DIR_FORWARD (1<<3)
#define DIR_BACKWARD (1<<2)
#define DIR_LEFT (1<<1)
#define DIR_RIGHT (1<<0)

void odetect_init(void);
int odetect_getDirection(int direction);
uint8_t odetect_getRaw(void);

#endif
