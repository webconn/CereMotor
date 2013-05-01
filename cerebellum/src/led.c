#include <cerebellum/led.h>

void led_init(void)
{
    // enable clock
    RCC_APB2PeriphClockCmd(CONFIG_LED1_RCC | CONFIG_LED2_RCC | CONFIG_LED3_RCC, ENABLE);

    // init gpio
    GPIO_InitTypeDef led = {
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_2MHz,
    };

    led.GPIO_Pin = (1<<CONFIG_LED1_PIN);
    GPIO_Init(CONFIG_LED1_GPIO, &led);

    led.GPIO_Pin = (1<<CONFIG_LED2_PIN);
    GPIO_Init(CONFIG_LED2_GPIO, &led);

    led.GPIO_Pin = (1<<CONFIG_LED3_PIN);
    GPIO_Init(CONFIG_LED3_GPIO, &led);
}

void led_on(uint32_t led)
{
    switch(led)
    {
        case 1:
            GPIO_SetBits(CONFIG_LED1_GPIO, (1<<CONFIG_LED1_PIN));
            break;
        case 2:
            GPIO_SetBits(CONFIG_LED2_GPIO, (1<<CONFIG_LED2_PIN));
            break;
        case 3:
            GPIO_SetBits(CONFIG_LED3_GPIO, (1<<CONFIG_LED3_PIN));
    }
}

void led_off(uint32_t led)
{
    switch(led)
    {
        case 1:
            GPIO_ResetBits(CONFIG_LED1_GPIO, (1<<CONFIG_LED1_PIN));
            break;
        case 2:
            GPIO_ResetBits(CONFIG_LED2_GPIO, (1<<CONFIG_LED2_PIN));
            break;
        case 3:
            GPIO_ResetBits(CONFIG_LED3_GPIO, (1<<CONFIG_LED3_PIN));
    }
}

void led_write(int led, int val)
{
    if(val)
        led_on(led);
    else
        led_off(led);
}
