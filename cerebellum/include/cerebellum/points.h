#ifndef CEREBELLUM_POINTS_H
#define CEREBELLUM_POINTS_H

#include <stm32f10x.h>

#include <cerebellum/movement.h>
#include <cerebellum/deltacoords.h>

#define ZERO_TWO_PI 1
#define MINUS_PI_PLUS_PI 2

typedef struct {
    int32_t x;
    int32_t y;
    float angle;
} point_t;

point_t * point_getPosition(void);
void point_setPosition(point_t * position);
void point_goto(point_t * destination); // just go to the point
void point_trace(point_t * destination, int32_t radius); // create a path and follow it
float point_optimiseAngle(float angle, uint32_t segment);

#endif
