#ifndef _CEREBELLUM_LED_H
#define _CEREBELLUM_LED_H

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <robots/config.h>

void led_init(void);
void led_on(uint32_t led);
void led_off(uint32_t led);
void led_write(uint16_t led, uint16_t val);

#endif
