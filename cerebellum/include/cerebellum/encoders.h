/**
 * @file include/cerebellum/encoders.h
 * @brief Headers file for Encoders API
 * @author WebConn
 * @date 5 Jan 2013
 */

#include <stm32f10x.h>

typedef struct{
    int32_t value; // counted value
    int32_t delta; // value of the last measure = encoder speed
    int32_t rawDelta;
} encState_t;

void encoders_init(void);
void encoders_calibrate(void);
encState_t * encoder_get(uint8_t encoder);
void encoder_reset(uint8_t encoder);
int32_t encoder_getDelta(uint8_t encoder);
int32_t encoder_getPath(uint8_t encoder);

void encoders_parser(void); // encoder updater, must be running in the timer
