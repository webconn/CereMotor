#include <pb/servo.h>

void servo_write(uint8_t servo, uint8_t value)
{
    uart_send(CONFIG_PB_UART, 's');
    uart_send(CONFIG_PB_UART, servo);
    uart_send(CONFIG_PB_UART, value);
}
