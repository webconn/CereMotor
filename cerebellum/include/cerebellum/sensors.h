#ifndef CEREBELLUM_SENSORS_H
#define CEREBELLUM_SENSORS_H

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

/**
 * Sensor operating modes
 */
#define SENSOR_PASSIVE_GND 1
#define SENSOR_PASSIVE_VCC 2
#define SENSOR_ACTIVE_GND 3
#define SENSOR_ACTIVE_VCC 4
#define SENSOR_ANALOG 5

typedef struct {
    GPIO_TypeDef * gpio;
    uint16_t pin;
    uint32_t mode;
} sensor_t;

void sensor_init(sensor_t * sensor, uint32_t rcc);
void sensor_addInterrupt(sensor_t * sensor);
uint32_t sensor_read(sensor_t * sensor);

#endif
