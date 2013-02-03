#include <sat/driver.h>
#include <sat/uart.h>
#include <sat/led.h>
#include <sat/tasks.h>
#include <sat/encoder.h>

#include <sat/receiver.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

volatile uint8_t max_speed = 255; // maximum speed (calibrated)

int main(void)
{
    // initialisation
    led_init();
    uart_init();
    encoder_init();
    driver_init();
 
    max_speed = eeprom_read_byte((uint8_t *) 1); // get calibrated value for Max_speed
    
    uart_send_byte('R');

    sei();

    DDRB &= ~(1<<1);
    PORTB |= (1<<1);

    while(1)
    {
        if(PINB & (1<<1))
            led_on();
        else
            led_off();
    }

    return 0;
}
