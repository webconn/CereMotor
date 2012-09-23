#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define E_FORWARD 1
#define E_BACKWARD 2


void encoder_init(void);

uint8_t encoder_get_speed(void);
uint8_t encoder_get_dir(void);
uint32_t encoder_get_length(void);
int32_t encoder_get_rellength(void);

void encoder_save_calibration(void);

void encoder_reset(void);

#endif
