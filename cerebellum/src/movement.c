#include <cerebellum/movement.h>
#include <cerebellum/chassis.h>
#include <cerebellum/pid.h>
#include <cerebellum/deltacoords.h>
#include <cerebellum/encoders.h>

#include <robots/config.h>

// moveMode contains number which describes movement
// algorithm: changing angle or move by line
// 1 - go by line
// 2 - change angle

int moveMode = 0;

inline void _move_stay(void);
inline void _move_line(void);
inline void _move_rotate(void);

// Tick function - running freely in SysTick and updating configuration
void move_tick(void)
{
    switch(moveMode)
    {
        case 0: // robot saves its position
            _move_stay();
            break;
        case 2: // robot moving
        case 1: // robot stopping
            _move_line();
            break;
        case 3: // robot rotating
        case 4: // robot stopping
            _move_rotate();
            break;
    }
}

// Stay tick function
inline void _move_stay(void)
{
    // To save robot's coordinates, we should block motors
    // and save this state
    chassis_break(CONFIG_PWM_ACCURACY, CONFIG_PWM_ACCURACY); // PWM at maximum
}

// Line movement tick function

uint16_t _movePWM, _moveAcc;
uint16_t _destPWM;
uint32_t _accPath = 0, _destPath;
int32_t lastSpeed = 0;

int32_t leftPWM, rightPWM;

/**
 * Move by line tick
 */
inline void _move_line(void)
{
    // Stabilisation algo depends on user selection - defined

    // 1. Get paths by both encoders
    int32_t leftPath = encoder_getPath(0);
    int32_t rightPath = encoder_getPath(1);

    int32_t aripPath = (leftPath + rightPath) >> 1; // sum and divide by 2

    int32_t leftSpeed = encoder_getDelta(0);
    int32_t rightSpeed = encoder_getDelta(1);

    int32_t acceleration = ((leftSpeed + rightSpeed) >> 1) - lastSpeed;
    lastSpeed = (leftSpeed + rightSpeed) >> 1;

    // Speed edjes: when starting and when stopping
    if(moveMode == 2) // normal operation (no brakes)
    {
        if(_movePWM < _destPWM) // acceleration
        {
            _movePWM += _moveAcc;
        }
        else // start normal operaion
        {
            _movePWM = _destPWM;
        }

        // Check real acceleration; if it less oq equal zero, save _accPath
        if(acceleration <= 0 && !_accPath)
        {
            _accPath = (aripPath * 5) >> 3;
        }

        // Check if we need to brake
        if((_destPath - aripPath) <= _accPath || (_destPath - aripPath) <= aripPath)
        {
            moveMode = 1;
        }
    }
    else if(moveMode == 1)
    {
        // At this stage we need to control encoders speed

        if(leftSpeed > 5 && rightSpeed > 5) // speed down while we should move
            _movePWM -= _moveAcc;

        if(aripPath >= _destPath)
        {
            moveMode = 0; // stop engines
            _movePWM = 0;
            _accPath = 0;
            _move_stay(); // stop right now, I said!
            return;
        }

    }

    // Stabilisation by encoders path
    #ifdef CONFIG_STAB_ALGO_PATH
        int32_t error = pid_calculateLinError(leftPath, rightPath);
    #endif

    // Stabilisation by angle
    #ifdef CONFIG_STAB_ALGO_ANGLE
        float angle = getAngle();
        int32_t error = (int32_t) ((angle - origAngle) * CONFIG_STAB_ALGO_ANGLE_K); // or change sign
    #endif

    // 3. Updating PID data (PWM overload calculated in this function)
    pid_update(error, _movePWM, &leftPWM, &rightPWM);

    // 4. Write data to chassis
    chassis_write(leftPWM, rightPWM);
}

int32_t move_getPWM(uint8_t val)
{
    if(val)
        return rightPWM;
    else
        return leftPWM;
}

/**
 * Rotate tick
 */
inline void _move_rotate(void)
{

}

/**
 * Move by line initializer
 */
void move_line(uint16_t pwm, uint16_t acceleration, uint32_t path)
{
    // To go by line, we need to run PID algorithm
    // stabilising paths or angle
    
    // 0. Clear encoder data
    encoder_reset(0);
    encoder_reset(1);

    // 1. Set data
    _destPWM = pwm;
    _moveAcc = acceleration;
    _destPath = path;

    // 2. Reset PID data
    pid_reset();

    // 3. Run algo in background
    moveMode = 2;
}

/**
 * Emergency stop
 */
void move_stop(void)
{

}

/**
 * Moving pause
 */

void move_pause(void)
{

}

/**
 * Moving continue
 */

void move_continue(void)
{

}

/**
 * Check movement system business
 */
int move_isBusy(void)
{
    return (moveMode > 0);
}

/**
 * Rotate initializer
 */
void move_rotate(int16_t pwm, float angle)
{
    // 1. Clear encoders data
}
