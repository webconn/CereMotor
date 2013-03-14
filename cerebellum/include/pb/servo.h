#ifndef PERIPH_SERVO_H
#define PERIPH_SERVO_H

#include <stm32f10x.h>
#include <cerebellum/uart.h>

void servo_write(uint8_t servo, uint8_t value);

#endif
