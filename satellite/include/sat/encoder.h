#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define E_FORWARD 1
#define E_BACKWARD 2

volatile uint32_t length;
volatile int32_t rellength;

void encoder_init(void);

uint8_t encoder_get_speed(void);
uint8_t encoder_get_dir(void);

void encoder_save_calibration(void);

void encoder_reset(void);

#endif
