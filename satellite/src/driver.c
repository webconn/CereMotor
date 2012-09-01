#include <sat/driver.h>

#include <avr/io.h>

void driver_init(void)
{
    // For driver communication, we use PORTB by default
    // Configuring pins to output
    DDRB |= (1<<4)|(1<<3)|(1<<2);

    // For PWM feature, we use Timer1
    // Configuring timer
    // Compare ch.A, PWM Phase Correct, 10-bit
    TCCR1A = (1<<COM1B1)|(1<<WGM11)|(1<<WGM10);
    TCCR1B = (1<<CS11)|(1<<CS10); // clock divide by 64
}

void driver_set_dir(uint8_t dir)
{
    // Variable dir in this function - one of consts: FORWARD or BACKWARD
    // This consts contains a ready bitmask for I/O port
    // Simply write it to the port!
    PORTB |= dir<<2;
    PORTB &= 0xFF & (dir<<2);
}

void driver_set_speed(uint16_t speed)
{
    // Formally, to set speed of motor, we need to change PWM length
    // Change PWM is to change timer compare register :)
    OCR1B = speed;
}

void driver_stop(void)
{
    // To stop motor, we will make short connection of wires
    // To make it, dir pins must be equal
    // Let's make it equals 1
    PORTB |= 3<<2;
}
