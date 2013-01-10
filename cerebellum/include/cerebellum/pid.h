/**
 * @file cerebellum/pid.h
 * @author WebConn
 * @version Rev.1
 * @date 24-Nov-12
 * @brief This file contains functions prototypes of speed stabilisation system of CereMotor
 */

#ifndef CEREBELLUM_PID_H
#define CEREBELLUM_PID_H

#include <math.h>

#define mmToTicks(m) ((uint32_t) ((m*CONFIG_ENC_RESOLUTION)/(2*CONFIG_ENC_WHEEL_RADIUS*M_PI)))

#include <stm32f10x.h>
#include <cerebellum/robot.h>

typedef struct{
    // Constants
    int32_t p_gain;
    int32_t i_rgain;
    int32_t d_rgain;

    int32_t i_max;
    int32_t i_min;

    // Algo memory
    int32_t i_mem;
    int32_t d_mem;
} pidConfig;

int32_t getRequiredPWM(int32_t requiredSpeed);
int32_t calculateLinError(int32_t speed1, int32_t speed2);
int32_t calculateRadError(int32_t speed1, int32_t speed2, int32_t radius);
void updatePID(int32_t error, int32_t requiredPWM, int32_t * pwm1, int32_t * pwm2);
void resetPID(void);
void configPID(pidConfig * data);

#endif
