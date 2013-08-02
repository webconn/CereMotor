#include <cerebellum/movement.h>
#include <stdio.h>

// ###################################################################################################
// Trying to create new stabilisators without garbage

// ################################################
// Global internal variables

// Pointers to PID regulators
pid_regulator_t * __path_pid, * __speed_pid;

// Pointer to current error calculator
// Default values - internal basic calculators
move_calculator_t __path_error_calculator = NULL;
move_errorCalculator_t __speed_error_calculator = &__move_speedErrorCalculator;
move_calculator_t __speed_calculator = NULL;
move_calculator_t __estop_handler = NULL;
move_stopCalculator_t __stop_calculator = NULL;

// Corrector values
int32_t __movePWM = 0; // Base PWM value for chassis
int32_t __moveSpeed = 0; // Base speed value (command for chassis)

int32_t __user_rqSpeed = 0; // Required speed value
int32_t __user_rqAcceleration = 0; // Required speed acceleration
int32_t __user_rqAccelerationDivider = 0; // Acceleration divider for very small accelerations

int16_t __user_dirLeft = 0, __user_dirRight = 0; // Required wheels direction (0 for none, 1 for forward, -1 for backward)
float __user_rqAngle = 0; // Required angle
int32_t __user_rqPath = 0; // Required path

int32_t __algo_rqSpeed = 0; // Speed set by algos
int32_t __algo_rqAcceleration = 0; // Acceleration set by algos
int32_t __algo_rqAccelerationDivider = 0; // Acceleration divider set by algos

int32_t __ebreak_rqSpeed = 0; // config for ebreak
int32_t __ebreak_rqAcceleration = 0;
int32_t __ebreak_rqAccelerationDivider = 0;

int32_t __stop_coefficient = 16;
int32_t __stop_baseLeft = 0;
int32_t __stop_baseRight = 0;
int32_t __stop_enableFlag = 0;

// End of Global internal variables
// ################################################
// Public functions description

/**
 * Register chassis PID regulator
 *
 * @param (pid_regulator_t *) pid Pointer to stabilisator
 */
void move_setPathPID(pid_regulator_t * pid)
{
    __path_pid = pid;
}

/**
 * Register speed PID regulator
 *
 * @param (pid_regulator_t *) pid Pointer to stabilisator
 */
void move_setSpeedPID(pid_regulator_t * pid)
{
    __speed_pid = pid;
}

/**
 * Register current speed value calculator
 * Used for generating speed values according to accelerations, etc
 *
 * @param move_calculator_t func Pointer to calculator function
 */
void move_setSpeedValueCalculator(move_calculator_t func)
{
    __speed_calculator = func;
}

/**
 * Register current speed error calculator
 *
 * @param move_errorCalculator_t func Pointer to calculator function
 */
void move_setSpeedErrorCalculator(move_errorCalculator_t func)
{
    __speed_error_calculator = func;
}

/**
 * Register current path error calculator
 *
 * @param move_calculator_t func Pointer to calculator function
 */
void move_setPathErrorCalculator(move_calculator_t func)
{
    __path_error_calculator = func;
}

/**
 * Register current stop 'zero position' calculator
 *
 * @param move_stopCalculator_t func Pointer to calculator function
 */
void move_setStopCalculator(move_stopCalculator_t func)
{
    __stop_calculator = func;
}

/**
 * Set user required stop coefficient
 */
void move_setStopCoefficient(int32_t coeff)
{
    __stop_coefficient = coeff;
}

/**
 * Set user required speed value
 */
void move_setRqSpeed(int32_t speed)
{
    __user_rqSpeed = speed;
}

/**
 * Set user required acceleration value
 */
void move_setRqAcceleration(int32_t acceleration)
{
    __user_rqAcceleration = acceleration;
}

/**
 * Set user required acceleration divider value
 */
void move_setRqAccelerationDivider(int32_t div)
{
    __user_rqAccelerationDivider = div;
}

/**
 * Set user required angle
 */
void move_setRqAngle(float angle)
{
    __user_rqAngle = angle;
}

/**
 * Set user required path
 */
void move_setRqPath(int32_t path)
{
    if(path < 0)
    {
        __user_dirLeft = -1;
        __user_dirRight = -1;
        __user_rqPath = -path;
    }
    else
    {
        __user_dirLeft = 1;
        __user_dirRight = 1;
        __user_rqPath = path;
    }
}

/**
 * Apply user values to algos
 */
void move_applyUserValues(void)
{
    __algo_rqSpeed = __user_rqSpeed;
    __algo_rqAcceleration = __user_rqAcceleration;
    __algo_rqAccelerationDivider = __user_rqAccelerationDivider;
    __moveSpeed = __speed_calculator();
}

/**
 * Set robot 'zero value'
 */
void move_setStopZeroValues(void)
{
    __stop_baseLeft = encoder_getPath(LEFT);
    __stop_baseRight = encoder_getPath(RIGHT);
}

/**
 * Lock robot chassis
 */
void move_stopEnable(void)
{
    __stop_enableFlag = 1;
}

/**
 * Unlock robot chassis
 */
void move_stopDisable(void)
{
    __stop_enableFlag = 0;
}

/**
 * Chassis activity handler
 * Simply returns 1 when required speed is not 0
 */
int move_isBusy(void)
{
    return __moveSpeed != 0;
}

// End of Public functions description
// ################################################
// Private (internal) functions description

/**
 * The most simple user-set speed calculator
 */
int32_t __move_speedCalculator_basic(void)
{
    return __user_rqSpeed;
}

/**
 * Basic user-set speed calculator
 * It stops movement by path
 */
int32_t __move_speedCalculator_path(void)
{
    if(__user_rqPath != 0 && __user_rqPath + __user_rqPath < encoder_getPath(0) + encoder_getPath(1))
        return 0;
    else
        return __user_rqSpeed; // move at user-set speed
}

/**
 * Advanced user-set speed calculator
 * Uses acceleration and calculates brake path
 */
int32_t __move_speedCalculator_acc(void)
{
    static int32_t speed = 0;
    static int32_t counter = 9999999;

    int32_t left_path = encoder_getPath(LEFT);
    int32_t right_path = encoder_getPath(RIGHT);
    int32_t left_speed = encoder_getDelta(LEFT);
    int32_t right_speed = encoder_getDelta(RIGHT);

    if(__user_dirLeft < 0)
    {
        left_path = -left_path;
        left_speed = -left_speed;
    }
    if(__user_dirRight < 0)
    {
        right_path = -right_path;
        right_speed = -right_speed;
    }
    
    if(__user_rqPath != 0) // if we need to move for defined value of path
    {
        // Calculate robot's path
        int32_t arip_path = (left_path + right_path) / 2;

        // Calculate current brake path
        int32_t brake_path = (speed * speed * (__algo_rqAccelerationDivider)) / (2 *  __algo_rqAcceleration);

        // 1. Calculate required speed
        // 1.1. Planned brake
        if(arip_path + arip_path >= __user_rqPath && __user_rqPath - arip_path <= brake_path)
        {
            __algo_rqSpeed = 1;
            led_on(3);
        }
        // 1.2. Final brake
        if(__user_rqPath <= arip_path)
        {
            speed = 0;
            counter = 999999;
            __algo_rqSpeed = 0;
            __user_rqPath = 0;

            return 0; // stop
        }
        led_off(3);
    }

    // 0. Internal counter for acceleration
    if(counter >= __algo_rqAccelerationDivider && speed != __algo_rqSpeed)
    {
        counter = 0; // now used for flag

        // Add acceleration
        if(speed > __algo_rqSpeed)
        {
            speed -= __algo_rqAcceleration;
        }
        else if(speed < __algo_rqSpeed)
        {
            speed += __algo_rqAcceleration;
            counter = 1;
        }

        // Check if speed is overloaded
        if((counter && speed > __algo_rqSpeed) ||
            (!counter && speed < __algo_rqSpeed))
            speed = __algo_rqSpeed;

        counter = 0;
    }
    else
        counter++;


    return speed;
}

/**
 * Basic speed error calculator
 *
 * If real speed is less than required, error will
 * be positive
 * In another case, error will be negative
 *
 * Error gain in this function is not necessary
 *
 * TODO: create an overload detector
 * When speed could'nt be reached because of maximum PWM,
 * function should send the signal
 */
int32_t __move_speedErrorCalculator(int32_t base)
{
    // Get real encoders speed
    int32_t enc_left, enc_right;
    
    enc_left = encoder_getDelta(LEFT);
    enc_right = encoder_getDelta(RIGHT);

    if(__user_dirLeft < 0)
        enc_left = -enc_left;

    if(__user_dirRight < 0)
        enc_right = -enc_right;

    return base - ((enc_left + enc_right) / 2);
}

/**
 * Stop error calculator
 * Keeps the robot on its 'zero position'
 */
void __move_stopCalculator_active(void)
{
    chassis_write(__stop_coefficient * (__stop_baseLeft - encoder_getPath(LEFT)), __stop_coefficient * (__stop_baseRight - encoder_getPath(RIGHT)));
}

void __move_stopCalculator_lock(void)
{
    chassis_break(8192, 8192);
}

/**
 * Angle-based chassis error calculator
 * This is a covering for PID algo's
 * angle calculator
 *
 * In this function, error gain is needed, because
 * angle error is too small values
 */
int32_t __move_angleErrorCalculator(void)
{
    if(__user_dirLeft < 0 && __user_dirRight < 0)
        return __path_pid->p_gain * (coords_getAngle() - __user_rqAngle);
    else
        return __path_pid->p_gain * (__user_rqAngle - coords_getAngle());
}

/**
 * Path-based chassis error calculator
 */
int32_t __move_pathsErrorCalculator(void)
{
    // Get encoder paths
    int32_t left = encoder_getPath(LEFT);
    int32_t right = encoder_getPath(RIGHT);

    // Return error
    if(__user_dirLeft < 0 && __user_dirRight < 0)
        return right - left;
    else
        return left - right;
}

int32_t __move_pathErrorCalculator_stay(void)
{
    return encoder_getPath(LEFT) - encoder_getPath(RIGHT);
}

/**
 * Wall-sensors chassis error calculator
 */
int32_t __move_pathErrorCalculator_wall(void)
{
    return 0;
}

/**
 * Main corrector function
 * Receives position error and correct PWM values
 */
void __move_correct(void)
{
    // 0. Calculate required speed
    __moveSpeed = __speed_calculator();

    int32_t lPWM, rPWM;
    if(__moveSpeed != 0 && !__stop_enableFlag)
    {
        // 1. Calculate base PWM according to speed error
        __movePWM = pid_correction(__speed_pid, __speed_error_calculator(__moveSpeed));

        // 2. Calculate position error and get PWMs
        pid_correctChassis(pid_correction(__path_pid, __path_error_calculator()), __movePWM, &lPWM, &rPWM);

        // 3. Apply PWMs to chassis according to direction flags

        if(__user_dirLeft < 0)
            lPWM = -lPWM;
        if(__user_dirRight < 0)
            rPWM = -rPWM;

        chassis_write(lPWM, rPWM);
    }
    else if(__stop_enableFlag)
    {
        // Stabilisation of "zero point"
        __stop_calculator();
    }
}
