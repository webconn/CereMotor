#ifndef CEREBELLUM_MOVEMENT
#define CEREBELLUM_MOVEMENT

#include <stm32f10x.h>

#include <cerebellum/chassis.h>
#include <cerebellum/pid.h>
#include <cerebellum/deltacoords.h>
#include <cerebellum/encoders.h>
#include <cerebellum/sensors.h>
#include <cerebellum/led.h>

#include <robots/config.h>

#define LEFT 1
#define RIGHT 0

#define PI 3.141593


// Functions
void move_initLimiters(sensor_t * lim_l, sensor_t * lim_r);
void move_tick(void);
void move_line(uint32_t pwm, uint32_t acceleration, uint32_t path);
void move_rotate(uint32_t path, uint32_t acceleration, float angle);
void move_refreshAngle(void);
void move_wall(uint32_t pwm, uint32_t acceleration, uint32_t path);

int32_t move_getPWM(uint8_t val);
int move_isBusy(void);

void move_setMinBrakeDelta(int32_t value);
int32_t move_getBrakePath(void);

#endif
