#include <cerebellum/pid.h>

// If left speed is greater than right speed, function returns positive value
// In another case - negative
int32_t pid_errorChassisSpeed(pid_regulator * pid, int32_t speed_l, int32_t speed_r)
{
    return pid->p_gain * (speed_l - speed_r);
}

int32_t pid_errorChassisAngle(pid_regulator * pid, float realAngle, float rqAngle)
{
    return pid->p_gain * (rqAngle - realAngle); // TODO: change values in case of wrong error
}

void pid_correctChassis(int32_t correction, int32_t base,  int32_t * pwm_l, int32_t * pwm_r)
{
    // Correction is positive when robot wrongly directed to right
    // So, remove correction from left and add it to the right
    *pwm_l = base - correction;
    *pwm_r = base + correction;

    // Check PWM for maximums and minimums
    if(*pwm_l > CONFIG_PWM_ACCURACY)
        *pwm_l = CONFIG_PWM_ACCURACY;
    else if(*pwm_l < -CONFIG_PWM_ACCURACY)
        *pwm_l = -CONFIG_PWM_ACCURACY;

    if(*pwm_r > CONFIG_PWM_ACCURACY)
        *pwm_r = CONFIG_PWM_ACCURACY;
    else if(*pwm_r < -CONFIG_PWM_ACCURACY)
        *pwm_r = -CONFIG_PWM_ACCURACY;
}

int32_t pid_correction(pid_regulator * pid, int32_t error)
{
    error /= 8; // this is for more accuracy

    // 1. Calculating integral
    pid->i_mem += error;

    if(pid->i_mem > pid->i_max)
        pid->i_mem = pid->i_max;
    else if(pid->i_mem < pid->i_min)
        pid->i_mem = pid->i_min;

    int32_t integral = pid->i_mem;

    // 2. Calculating derivative
    int32_t derivative = pid->d_mem;
    pid->d_mem = error;

    // 3. Calculating full sentence
    return error + (integral / pid->i_rgain) - (derivative / pid->d_rgain);
}

void pid_reset(pid_regulator * pid)
{
    pid->i_mem = 0;
    pid->d_mem = 0;
}
