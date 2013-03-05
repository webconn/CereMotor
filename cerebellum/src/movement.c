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
inline void _move_stab(void);

// Tick function - running freely in SysTick and updating configuration
void move_tick(void)
{
    if(moveMode == 0)
        _move_stay();
    else
        _move_stab();
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
int32_t _midAcc = 0;
uint32_t sign = 0;

float destAngle = 0, startAngle = 0, deltaAngle = 0;
float _accAngle = 0;

int32_t leftPWM, rightPWM;

/**
 * Move by line tick
 */
inline void _move_stab(void)
{
    // Stabilisation algo depends on user selection - defined

    // 1. Get paths by both encoders
    int32_t leftPath = encoder_getPath(0);
    int32_t rightPath = encoder_getPath(1);

    int32_t aripPath = (leftPath + rightPath) / 2; // sum and divide by 2

    int32_t leftSpeed = encoder_getDelta(0);
    int32_t rightSpeed = encoder_getDelta(1);

    float angle = getAngle();

    // Specially for angle stabilisation
    if(moveMode > 2)
    {
        if(sign)
        {
            rightSpeed = -rightSpeed;
            rightPath = -rightPath;
        }
        else
        {
            leftSpeed = -leftSpeed;
            leftPath = -leftPath;
        }
    }

    int32_t acceleration = ((leftSpeed + rightSpeed) / 2) - lastSpeed;
    
    if(!_midAcc)
        _midAcc = acceleration;
    else if(!_accPath || !_accAngle)
        _midAcc = (_midAcc + acceleration) / 2;

    lastSpeed = (leftSpeed + rightSpeed) / 2;

    // Speed edjes: when starting and when stopping
    if(moveMode == 2 || moveMode == 4) // normal operation (no brakes)
    {
        if(_movePWM < _destPWM) // acceleration
        {
            _movePWM += _moveAcc;
        }
        else // start normal operaion
        {
            _movePWM = _destPWM;
        }

        // Check if we need to brake
        if(moveMode == 2)
        {
            if((_destPath - aripPath) <= _accPath || (_destPath - aripPath) <= aripPath)
            {
                moveMode = 1;
            }
            
            // Check acceleration: if accelerated, take the measure
            if(acceleration <= 0 && !_accPath)
            {
                _accPath = aripPath;
            }
        }
        else
        {   
            if((destAngle - angle) <= _accAngle || (destAngle - angle) <= deltaAngle)
            {
                moveMode = 3;
            }

            if(acceleration <= 0 && !_accAngle)
            {
                _accAngle = angle - startAngle;
            }
        }
    }
    else if(moveMode == 1 || moveMode == 3)
    {
        // At this stage we need to control encoders speed

        if(leftSpeed > 5 && rightSpeed > 5 && acceleration >= -_midAcc) // speed down while we should move
            _movePWM -= _moveAcc;

        
        // If we reached end, stop engines and shut algo down
        if(moveMode == 1)
        {
            if(aripPath >= _destPath)
            {
                moveMode = 0; // stop engines
                _movePWM = 0;
                _accPath = 0;
                _midAcc = 0;
                _move_stay(); // stop right now, I said!
                return;
            }
        }
        else
        {
            if(angle >= destAngle)
            {
                moveMode = 0;
                _movePWM = 0;
                _accAngle = 0;
                _midAcc = 0;
                _move_stay();
                return;
            }
        }

    }

    // Stabilisation by encoders path
    int32_t error = pid_calculateLinError(leftPath, rightPath);

    // 3. Updating PID data (PWM overload calculated in this function)
    pid_update(error, _movePWM, &leftPWM, &rightPWM);

    // 4. Write data to chassis
    if(moveMode < 3) // for linear moving
    {
        chassis_write(leftPWM, rightPWM);
    }
    else // for rotation
    {
        if(sign)
        {
            chassis_write(leftPWM, -rightPWM);
        }
        else
        {
            chassis_write(-leftPWM, rightPWM);
        }
    }
}

// Debug purposes
int32_t move_getPWM(uint8_t val)
{
    if(val)
        return rightPWM;
    else
        return leftPWM;
}

/**
 * Move by line initializer
 */
void move_line(uint32_t pwm, uint32_t acceleration, uint32_t path)
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
 * Rotate initializer
 */
void move_rotate(uint32_t pwm, uint32_t acceleration, float dAngle)
{
    // 0. Clear encoders and PWM
    pid_reset();
    encoder_reset(0);
    encoder_reset(1);

    // 1. Set data
    deltaAngle = dAngle;
    startAngle = getAngle();
    destAngle = deltaAngle + startAngle;
    deltaAngle /= 2; // divide for path comparator - oops...

    _destPWM = pwm;
    _moveAcc = acceleration;

    if(dAngle > 0) sign = 1;
    else sign = 0;
    
    // 2. Run algo in background
    moveMode = 4;
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
