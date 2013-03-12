#ifndef _CEREBELLUM_LED_H
#define _CEREBELLUM_LED_H

#include <stm32f10x.h>

void led_init(void);
void led_on(uint32_t led);
void led_off(uint32_t led);
void led_write(uint32_t led, uint32_t val);

#endif
