#ifndef LED_H
#define LED_H

#include <avr/io.h>

#define led_init() DDRD |= (1<<6)
#define led_on() PORTD |= (1<<6)
#define led_off() PORTD &= ~(1<<6)

#endif
