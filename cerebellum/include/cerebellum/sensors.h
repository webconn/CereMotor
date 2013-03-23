#ifndef CEREBELLUM_SENSORS_H
#define CEREBELLUM_SENSORS_H

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_adc.h>
#include <misc.h>
#include <cerebellum/snippet.h>
#include <robots/config.h>

/**
 * Sensor operating modes
 */
#define SENSOR_PASSIVE_GND 1
#define SENSOR_PASSIVE_VCC 2
#define SENSOR_ACTIVE_GND 3
#define SENSOR_ACTIVE_VCC 4
#define SENSOR_ANALOG 5
#define SENSOR_ANALOG_THRESHOLD_HIGH 6
#define SENSOR_ANALOG_THRESHOLD_LOW 7

typedef struct {
    GPIO_TypeDef * gpio;
    uint16_t pin;
    uint32_t mode;
    uint8_t channel; // for ADC
    uint8_t chid; // keep it - value is for library
    uint16_t threshold;
} sensor_t;

void sensor_init(void); // pre-launch library initialisation (DMA, ADC)
void sensor_add(sensor_t * sensor);
void sensor_addInterrupt(sensor_t * sensor);
uint32_t sensor_read(sensor_t * sensor);

#endif
