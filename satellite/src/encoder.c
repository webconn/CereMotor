#include <sat/encoder.h>
#include <sat/tasks.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

volatile uint32_t length = 0; // absolute length (without dir)
volatile int32_t rellength = 0; // relative length (with dir)
volatile uint16_t speed = 0; // current speed
volatile uint16_t _speed = 0; // unparsed speed

void _encoder_tick(void);

void encoder_init(void)
{
    // Initialisation of encoder is init of INT0 and configuring PD3
    DDRD &= 0xFF & ~(1<<3); // channel B to input
    MCUCR = (1<<ISC01)|(1<<ISC00); // sense to rising edge on INT0
    GIMSK = (1<<INT0); // interrupt enable
    task_add(&_encoder_tick, 0); // add _encoder_tick to task manager
}

uint16_t encoder_get_speed(void)
{
    return speed;
}

uint32_t encoder_get_length(void)
{
    return length;
}

int32_t encoder_get_rellength(void)
{
    return rellength;
}

void encoder_reset(void)
{
    length = 0;
    rellength = 0;
}

ISR(INT0_vect)
{
    length++;
    _speed++;
    if((PIND & (1<<3)) == 0) rellength++;
    else rellength--;
}

void _encoder_tick(void)
{
    task_add(&_encoder_tick, 10); // run this task every 10 ms
    speed = _speed;
    _speed = 0;
}
