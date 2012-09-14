#include <sat/driver.h>
#include <sat/uart.h>
#include <sat/led.h>
#include <sat/tasks.h>
#include <sat/encoder.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

volatile uint8_t max_speed = 255; // maximum speed (calibrated)

volatile uint8_t stat = 0;
uint8_t i;

void Rx_handler(void) // command reader
{
    char byte = UDR;
    uart_send_byte(byte);

    if(stat == 0) // if this is new command
    {
        if(byte == 'd' && byte == 's')
        {
            stat = byte;
        }
        else if(byte == 'c') // reset encoder
        {
            encoder_reset();
        }
        else if(byte == 'e') // get encoder directory
        {
            uart_send_byte((char) encoder_get_dir());
        }
        else if(byte == 'r') // get driver directory
        {
            uart_send_byte((char) driver_get_dir());
        }
        else if(byte == 'm') // get max speed
        {
            uart_send_byte((char) max_speed);
        }
        else if(byte == 'v') // get current velocity
        {
            uart_send_byte((char) encoder_get_speed());
        }
        else if(byte == 'l') // get length
        {
            for(i=0; i!=4; i++)
                uart_send_byte(*((char *) &length + i));
        }
        else if(byte == 'p') // get relative length (path)
        {
            for(i=0; i!=4; i++)
                uart_send_byte(*((char *) &length + i));
        }
    }
    else // if we continue receive commands
    {
        if(stat == 'd') // set direction
            driver_set_dir(byte);
        //else if(stat == 's') // set speed
            // TODO: set speed handler
        stat = 0;
    }
}

int main(void)
{
    // initialisation
    led_init();
    uart_init();
    task_init();
    encoder_init();
    driver_init();
    
    max_speed = eeprom_read_byte((uint8_t *) 1); // get calibrated value for Max_speed

    uart_send_byte('R');
    uart_set_rx_handler(&Rx_handler);
    led_on();

    sei();

    while(1)
    {
        task_manager();
    }

    return 0;
}
