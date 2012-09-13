#include <sat/driver.h>
#include <sat/uart.h>
#include <sat/led.h>
#include <sat/tasks.h>
#include <sat/encoder.h>

#include <avr/interrupt.h>
#include <util/delay.h>

void Rx_handler(void)
{
    uint8_t i = 0;
    char * buf = uart_get_rx_buffer();
    uart_send_string("Received: ");
    while(i != CONFIG_SAT_UART_RXBUFFER)
    {
        uart_send_byte(buf[i++]);
    }
    uart_flush_rx_buffer();
}


void Enc_handler(void)
{
    task_add(&Enc_handler, 40);
    cli();
    uart_send_string("Speed: ");
    uart_send_string(to_str(encoder_get_speed()));
    uart_send_byte('\r');
    sei();
}

int main(void)
{
    // initialisation
    led_init();
    uart_init();
    task_init();
    encoder_init();
    
    uart_send_string("Ready.\n\n\r");

    sei();

    Enc_handler();

    while(1)
    {
        task_manager();
    }

    return 0;
}
