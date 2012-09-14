#include <sat/encoder.h>
#include <sat/tasks.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdint.h>


volatile uint32_t length = 0; // absolute length (without dir)
volatile int32_t rellength = 0; // relative length (with dir)

volatile uint8_t speed = 0; // current speed
volatile uint16_t _speed = 0; // unparsed speed
volatile uint8_t dir = 0; // 0 - stopped, 1 - forward, 2 - backward

volatile uint8_t _skips = 0; // timer skips to count speed

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

    // Read calibration data
    _skips = eeprom_read_byte((uint8_t *) 0);
}

uint8_t encoder_get_dir(void)
{
    if(speed) return dir;
    else return 0;
}

uint8_t encoder_get_speed(void)
{
    return speed;
}

void encoder_reset(void)
{
    length = 0;
    rellength = 0;
}

void encoder_save_calibration(void)
{
    eeprom_write_byte((uint8_t *) 0, _skips);
}

ISR(INT1_vect)
{
    length++;
    _speed++;
    if((PIND & (1<<4)) == 0) 
    {
        dir = 2;
        rellength--;
    }
    else
    {
        dir = 1;
        rellength++;
    }
}

volatile uint8_t __skipped = 0;

ISR(TIMER0_OVF_vect)
{
    speed = _speed;
    _speed = 0;
}
