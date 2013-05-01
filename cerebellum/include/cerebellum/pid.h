/**
 * @file cerebellum/pid.h
 * @author WebConn
 * @version Rev.1
 * @date 24-Nov-12
 * @brief This file contains functions prototypes of speed stabilisation system of CereMotor
 */

#ifndef CEREBELLUM_PID_H
#define CEREBELLUM_PID_H

#include <stm32f10x.h>
#include <cerebellum/robot.h>
#include <robots/config.h>

typedef struct {
    // Constants
    int32_t p_gain;
    int32_t i_rgain;
    int32_t d_rgain;

    int32_t i_max;
    int32_t i_min;

    // Algo memory
    int32_t i_mem;
    int32_t d_mem;
} pid_regulator_t;

int32_t pid_errorChassisSpeed(pid_regulator_t * pid, int32_t speed_l, int32_t speed_r);
int32_t pid_errorChassisAngle(pid_regulator_t * pid, float realAngle, float rqAngle);
void pid_correctChassis(int32_t correction, int32_t base,  int32_t * pwm_l, int32_t * pwm_r);

/**
 * PID correction calculating
 *
 * @param (pid_regulator_t *) pid PID state
 * @param int32_t error Calculated error
 *
 * @return int32_t Correction value (ex. for macros correctChassis)
 */
int32_t pid_correction(pid_regulator_t * pid, int32_t error);

/**
 * Reset PID specific values
 *
 * @param (pid_regulator_t *) pid PID state
 */
void pid_reset(pid_regulator_t * pid);

#endif
