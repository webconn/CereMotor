#include <sat/encoder.h>
#include <sat/tasks.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

volatile uint32_t length = 0; // absolute length (without dir)
volatile int32_t rellength = 0; // relative length (with dir)
volatile uint16_t speed = 0; // current speed
volatile uint16_t _speed = 0; // unparsed speed


void encoder_init(void)
{
    // Firstly, let's launch the timer
    // It helps to calculate speed more correctly
    TCCR0B = (1<<CS02)|(1<<CS00); // clk/1024 prescaler
    TIMSK |= (1<<TOIE0);
    
    // Initialisation of encoder is init
    // of INT1 and configuring PD2
    DDRD &= 0xFF & ~(1<<4); // channel A to input
    MCUCR = (1<<ISC11); // sense to falling edge on INT1
    GIMSK = (1<<INT1); // interrupt enable
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

ISR(INT1_vect)
{
    length++;
    _speed++;
    if((PIND & (1<<4)) == 0) rellength--;
    else rellength++;
}

ISR(TIMER0_OVF_vect)
{
    speed = _speed;
    _speed = 0;
}
