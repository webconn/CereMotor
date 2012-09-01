#include <sat/uart.h>

#include <avr/io.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <sat/led.h>
#include <sat/tasks.h>

void (* _rx_handler)(void) = 0;
char _rx_buffer[CONFIG_SAT_UART_RXBUFFER];
uint8_t _rx_buffer_index = 0;

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
    UCSRC = (1<<UCSZ1)|(1<<UCSZ0); // 8-bit, 1 stop bit, no parity
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

char * uart_get_rx_buffer(void)
{
    return _rx_buffer;
}

void uart_flush_rx_buffer(void)
{
    uint8_t i = 0;
    while(i != CONFIG_SAT_UART_RXBUFFER)
        _rx_buffer[i++] = 0;
}

ISR(USART_RX_vect)
{
    char rx_byte = UDR;
    if(rx_byte != 0 && _rx_buffer_index != CONFIG_SAT_UART_RXBUFFER) // if the end of line is not reached
        _rx_buffer[_rx_buffer_index++] = rx_byte; 
    else // if the end of line is reached or the buffer is full
    {
        _rx_buffer_index = 0;
        task_add(_rx_handler, 0);
    }
    
}
