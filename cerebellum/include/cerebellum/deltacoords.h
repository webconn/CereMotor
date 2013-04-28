/**
 * @file cerebellum/deltacoords.h
 * @author WebConn
 * @version Rev.1
 * @date 25-Nov-12
 * @brief This file contains function prototypes of coordinates computing system of CereMotor
 */

#ifndef CEREBELLUM_DELTACOORDS
#define CEREBELLUM_DELTACOORDS

/**
 * @note Warning! Floating point calculations! Trigonometry!
 *       It should be better to use MCU with DSP supports floating point calculation
 *       In another case we need to include math library from GCC
 */

#ifdef CONFIG_CEREBELLUM_USE_GCC_MATH
    #include <math.h>
#endif

#include <math.h>

#include <stm32f10x.h>
#include <cerebellum/robot.h>
#include <robots/config.h>

float getDeltaAngle(int32_t speed1, int32_t speed2);
int32_t getCentralSpeed(int32_t speed1, int32_t speed2);

int32_t getX(void);
int32_t getY(void);
float getAngle(void);

void updateX(int32_t x);
void updateY(int32_t y);
void updateAngle(float angle);

void updateCoords(int32_t speed1, int32_t speed2);


#endif
