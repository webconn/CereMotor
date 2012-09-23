#include <sat/led.h>
#include <avr/io.h>

void led_init(void)
{
    DDRD |= 1<<6;
}

void led_on(void)
{
    PORTD |= 1<<6;
}

void led_off(void)
{
    PORTD &= ~(1<<6);
}
