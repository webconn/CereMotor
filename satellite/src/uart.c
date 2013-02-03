#include <sat/uart.h>

#include <avr/io.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <sat/led.h>
#include <sat/tasks.h>

void (* _rx_handler)(void) = 0;

void uart_init(void)
{
    DDRD &= ~1;
    DDRD |= (1<<1);
    // Calculating baud rate
    UBRRH = (uint8_t) BDRT >> 8;
    UBRRL = (uint8_t) BDRT;
    // Configuring lines
    UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN); // enable Rx,Tx; Rx interrupt
    // Configuring interface
    UCSRC = (1<<UCSZ1)|(1<<UCSZ0); // 8-bit, 1 stop bit, even parity
}

void uart_send_byte(char byte)
{
    while(!(UCSRA & (1<<UDRE))); // waiting for UART ready
    UDR = byte; // send byte
}

void uart_send_string(char * string)
{
    while(*string != '\0')
    {
        uart_send_byte(*string);
        string++;
    }
}

void uart_set_rx_handler(void * handler)
{
    _rx_handler = handler;
}

