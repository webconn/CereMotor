#ifndef UART_H
#define UART_H

#define BDRT (F_CPU/16/CONFIG_SAT_UART_BAUDRATE - 1)

void uart_init(void);
void uart_send_byte(char byte);
void uart_send_string(char * string);
void uart_set_rx_handler(void * handler);
char * uart_get_rx_buffer(void);
void uart_flush_rx_buffer(void);

#endif
