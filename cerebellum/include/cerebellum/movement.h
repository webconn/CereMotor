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
#include <cerebellum/points.h>

#include <robots/config.h>

#define LEFT 1
#define RIGHT 0

// Refactored functions and defines
typedef int32_t (*move_calculator_t)(void); // sets move_calculator type that contains pointer to a function
typedef int32_t (*move_errorCalculator_t)(int32_t); // sets move error calculator
typedef void (*move_stopCalculator_t)(int32_t *, int32_t *); // set 'zero position' error calculator (raw, w/o PID)

void move_setPathPID(pid_regulator_t * pid);
void move_setSpeedPID(pid_regulator_t * pid);

void move_setSpeedValueCalculator(move_calculator_t func);
void move_setSpeedErrorCalculator(move_errorCalculator_t func);
void move_setPathErrorCalculator(move_calculator_t func);
void move_setStopCalculator(move_stopCalculator_t func);

void move_setRqSpeed(int32_t speed);
void move_setRqAcceleration(int32_t acceleration);
void move_setRqAccelerationDivider(int32_t div);
void move_setRqPath(int32_t path);
void move_setRqAngle(float angle);

void move_applyUserValues(void);

void move_setStopZeroValues(void);
void move_setStopCoefficient(int32_t coeff);

// General-purpose speed calculators
int32_t __move_speedCalculator_stop(void);
int32_t __move_speedCalculator_basic(void);
int32_t __move_speedCalculator_acc(void);

// Error calculators
int32_t __move_speedErrorCalculator(int32_t base);
int32_t __move_pathsErrorCalculator(void);
int32_t __move_angleErrorCalculator(void);
int32_t __move_pathErrorCalculator_stay(void);
int32_t __move_pathErrorCalculator_wall(void);

void __move_stopCalculator_basic(int32_t * lPWM, int32_t * rPWM);

// Corrector function
void __move_correct(void);

#endif
