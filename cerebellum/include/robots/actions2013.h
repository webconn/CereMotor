#ifndef ROBOTS_ACTIONS2013_H
#define ROBOTS_ACTIONS2013_H

#include <stm32f10x.h>
#include <cerebellum/servo.h>
#include <cerebellum/sensors.h>
#include <cerebellum/movement.h>

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

/**
 * Grips
 */
#ifndef LEFT
#define LEFT 1
#endif
#ifndef RIGHT
#define RIGHT 2
#endif

/**
 * Grip states
 */
#define OPEN 1
#define HOLD 2
#define UNHOLD 3

void actions_init(servo bpaw, servo spaw, servo elev, servo gl, servo gr, sensor_t * limit_h, sensor_t * limit_l);

void paw_move(uint16_t paw, uint16_t state);
void elevator_move(uint16_t state);
int elevator_moveCareful(uint16_t state);
void grip_set(uint16_t grip, uint16_t state);
void take_glass(uint16_t side, uint16_t elevator_state);

#endif
