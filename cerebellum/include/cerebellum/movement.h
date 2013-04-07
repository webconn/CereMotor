#ifndef CEREBELLUM_MOVEMENT
#define CEREBELLUM_MOVEMENT

#include <stm32f10x.h>

#include <cerebellum/chassis.h>
#include <cerebellum/pid.h>
#include <cerebellum/deltacoords.h>
#include <cerebellum/encoders.h>
#include <cerebellum/sensors.h>
#include <cerebellum/led.h>
#include <cerebellum/odetect.h>

#include <robots/config.h>

#define LEFT 1
#define RIGHT 0

#define PI 3.141593


// Functions
void move_initLimiters(sensor_t * lim_l, sensor_t * lim_r);
void move_initWallSensor(sensor_t * rf_front, sensor_t * rf_rear);
void move_tick(void);
void move_line(int32_t pwm, int32_t acceleration, int32_t path);
void move_rotate(int32_t path, int32_t acceleration, float angle);
void move_refreshAngle(void);
void move_wall(int32_t pwm, int32_t acceleration, int32_t path);
void move_toPoint(int32_t x, int32_t y, int32_t pwm, int stab);
void move_stop(void);
void move_pause(void);
void move_continue(void);

int32_t move_getPWM(uint8_t val);
int move_isBusy(void);
uint8_t move_getRelativeDirection(void);

void move_setMinBrakeDelta(int32_t value);
int32_t move_getBrakePath(void);
void move_free();

#endif
