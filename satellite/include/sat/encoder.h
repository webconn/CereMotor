#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);

uint16_t encoder_get_speed(void);
uint32_t encoder_get_length(void);
int32_t encoder_get_rellength(void);

void encoder_reset(void);

#endif
