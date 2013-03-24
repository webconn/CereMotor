#ifndef ROBOTS_ACTIONS2013_H
#define ROBOTS_ACTIONS2013_H

#include <stm32f10x.h>
#include <cerebellum/servo.h>
#include <cerebellum/sensors.h>

/**
 * Paw states
 */
#define OPEN 1
#define CLOSE 2
#define BLOW 3

/**
 * Paws
 */
#define BIG 1
#define SMALL 2

/**
 * Elevator states
 */
#define UP 1
#define DOWN 2

void actions_init(servo bpaw, servo spaw, servo elev, sensor_t * limit_h, sensor_t * limit_l);

void paw_move(uint16_t paw, uint16_t state);
void elevator_move(uint16_t state);

#endif
