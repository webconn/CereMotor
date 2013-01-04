#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#define LED_RCC RCC_APB2Periph_GPIOA
#define LED_GPIO GPIOA
#define LED_PIN 5

void led_init(void)
{
    // enable clock
    RCC_APB2PeriphClockCmd(LED_RCC, ENABLE);

    // init gpio
    GPIO_InitTypeDef led = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_Pin = (1<<LED_PIN)
    };

    GPIO_Init(LED_GPIO, &led);
}

void led_on(void)
{
    GPIO_SetBits(LED_GPIO, (1<<LED_PIN));
}

void led_off(void)
{
    GPIO_ResetBits(LED_GPIO, (1<<LED_PIN));
}

void led_write(int val)
{
    if(val)
        led_on();
    else
        led_off();
}
