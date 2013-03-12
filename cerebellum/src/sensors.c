#include <cerebellum/sensors.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>

void sensor_init(sensor_t * sensor, uint32_t rcc)
{
    // 0. Init RCC
    RCC_APB2PeriphClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef pin_GPIO;

    // 1. Set mode GPIO init structure
    switch(sensor->mode)
    {
        case SENSOR_PASSIVE_GND:
            pin_GPIO.GPIO_Mode = GPIO_Mode_IPU;
            break;
        case SENSOR_PASSIVE_VCC:
            pin_GPIO.GPIO_Mode = GPIO_Mode_IPD;
            break;
        case SENSOR_ANALOG:
            pin_GPIO.GPIO_Mode = GPIO_Mode_AIN;
            break;
        default:
            pin_GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }

    // 2. Set GPIO pin
    pin_GPIO.GPIO_Pin = sensor->pin;

    // 3. Init GPIO
    GPIO_Init(sensor->gpio, &pin_GPIO);
}

uint32_t sensor_read(sensor_t * sensor)
{
    if(sensor->mode == SENSOR_ANALOG)
    {
        // TODO: insert ADC parser
        return 0;
    }
    else if(sensor->mode == SENSOR_ACTIVE_GND || sensor->mode == SENSOR_PASSIVE_GND)
    {
        return GPIO_ReadInputDataBit(sensor->gpio, sensor->pin) == 0;
    }
    else // active and passive to VCC
    {
        return GPIO_ReadInputDataBit(sensor->gpio, sensor->pin) > 0;
    }
}
