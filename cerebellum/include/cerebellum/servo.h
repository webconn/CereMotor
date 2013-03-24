#ifndef INCLUDE_CEREBELLUM_H
#define INCLUDE_CEREBELLUM_H

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <misc.h>
#include <cerebellum/snippet.h>
#include <robots/config.h>

typedef uint32_t servo;

void servo_init(void);
servo servo_add(GPIO_TypeDef * gpio, uint8_t pin);
void servo_write(servo servo_id, uint16_t value);
void servo_set(servo servo_id, uint8_t value);
void servo_setLimits(servo servo_id, uint16_t min, uint16_t max);

#endif
