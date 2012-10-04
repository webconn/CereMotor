#include <sat/encoder.h>
#include <sat/tasks.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdint.h>
#include <sat/led.h>
#include <sat/driver.h>

volatile uint32_t length = 0; // absolute length (without dir)
volatile int32_t rellength = 0; // relative length (with dir)

volatile uint16_t speed = 0; // current speed
volatile uint16_t _speed = 0; // unparsed speed
volatile uint8_t dir = 0; // 0 - stopped, 1 - forward, 2 - backward

volatile uint8_t _skips = 0; // timer skips to count speed

volatile uint16_t rq_speed = 0; // required speed

volatile uint8_t _prop = 1;
volatile uint8_t _diff = 1;

uint8_t flag_auto = 0;

void set_auto(void)
{
    flag_auto = 1;
}

void set_manual(void)
{
    flag_auto = 0;
}

uint16_t rq_getspeed(void)
{
    return rq_speed;
}

void rq_setspeed(uint16_t spd)
{
    rq_speed = spd;
}

void rq_setprop(uint8_t p)
{
    _prop = p;
}

void rq_setdiff(uint8_t d)
{
    _diff = d;
}

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

int16_t error;
int16_t last_error;

ISR(TIMER0_OVF_vect)
{
    speed = _speed;
    _speed = 0;

    // Stabilisation algorhythm running if flag is set
    if(flag_auto)
    {
        error = rq_speed - speed;
        error = driver_get_speed() + _prop*(error >> 3) - _diff*(last_error >> 4);
        last_error = rq_speed - speed;
        if(error > 1024) error = 1024;
        else if(error < 0) error = 0;
        driver_set_speed(error);
    }
}
