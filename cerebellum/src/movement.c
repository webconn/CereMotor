#include <cerebellum/movement.h>
#include <cerebellum/chassis.h>
#include <cerebellum/pid.h>
#include <cerebellum/deltacoords.h>
#include <cerebellum/encoders.h>

#include <robots/config.h>

#define LEFT 1
#define RIGHT 0

// MinBrakeDelta is a calibrated value
int32_t MinBrakeDelta = 0;
int32_t BrakeStart = 0;

void move_setMinBrakeDelta(int32_t value)
{
    MinBrakeDelta = value;
}

int32_t move_getBrakePath(void)
{
    return BrakeStart;
}

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
    if(moveMode == 0)
        _move_stay();
    else if(moveMode < 3)
        _move_line();
    else
        _move_rotate();
}

// Stay tick function
inline void _move_stay(void)
{
    // To save robot's coordinates, we should block motors
    // and save this state
    chassis_break(CONFIG_PWM_ACCURACY, CONFIG_PWM_ACCURACY); // PWM at maximum
}

int32_t _movePWM, _moveAcc;
int32_t _destPWM;
int32_t _accPath = 0, _destPath;
int32_t lastSpeed = 0;
int32_t _midAcc = 0;
int32_t sign = 0;
int32_t _numMeasures = 0;

int32_t leftPWM, rightPWM;

inline void _move_line(void)
{
    // Stabilisation algo depends on user selection - defined

    // 1. Get paths by both encoders
    int32_t leftPath = encoder_getPath(1);
    int32_t rightPath = encoder_getPath(0);

    int32_t aripPath = (leftPath + rightPath) / 2; // sum and divide by 2

    int32_t leftSpeed = encoder_getDelta(1);
    int32_t rightSpeed = encoder_getDelta(0);

    int32_t acceleration = ((leftSpeed + rightSpeed) / 2) - lastSpeed;

    lastSpeed = (leftSpeed + rightSpeed) / 2;

    // Speed edjes: when starting and when stopping
    if(moveMode == 2) // normal operation (no brakes)
    {
        // 1. Calculate middle acceleration
        if(!_midAcc && acceleration > 0)
        {
            _midAcc = acceleration;
        }
        else if(acceleration > _midAcc)
        {
            _midAcc = acceleration;
        }

        // Increase PWM
        if(_movePWM < _destPWM) // acceleration
        {
            _movePWM += _moveAcc;
        }
        else // start normal operaion
        {
            _movePWM = _destPWM;
        }

        // Check if we need to brake
        if((_destPath - aripPath) <= _accPath || (_accPath == 0 && (_destPath - aripPath) <= aripPath))
        {
            moveMode = 1;
        }
        
        // Check acceleration: if accelerated, take the measure
        if(acceleration <= 0 && !_accPath && _movePWM == _destPWM)
        {
            _accPath = aripPath;
            _midAcc /= _numMeasures;
        }
    }
    else 
    {
        // At this stage we need to control encoders speed
        if(leftSpeed > 5 && rightSpeed > 5 && acceleration >= -_midAcc) // speed down while we should move
            _movePWM -= _moveAcc;

        
        // If we reached end, stop engines and shut algo down
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

    // Stabilisation by encoders path
    int32_t error = pid_calculateLinError(rightPath, leftPath);

    // 3. Updating PID data (PWM overload calculated in this function)
    pid_update(error, _movePWM, &leftPWM, &rightPWM);

    // 4. Write data to chassis
    chassis_write(leftPWM, rightPWM);
}

/**
 * Move by line tick
 */


int32_t flag_GetMinBrake = 0;
int32_t flag_CorrectorDirSet = 0;
int32_t MinBrakePath = 0;
int32_t lastDelta = 0;
int32_t numMeasures = 0;
int32_t deltaSpeed = 0;
int32_t rqSpeed = 0;
float rqAngle = 0;
int32_t _mdir = 0;

inline void _move_rotate(void)
{
    // Input:
    // _moveAcc: movement acceleration
    // _destPath: required wheel path
    // _destPWM: maximum PWM
    
    // Stage 1
    // Parse data from encoders
    // We need to get deltas and paths from
    // each encoder and change signs for compability
    // with basic PID regulator

    // Read data

    int32_t leftPath = encoder_getPath(LEFT);
    int32_t rightPath = encoder_getPath(RIGHT);

    int32_t leftDelta = encoder_getDelta(LEFT);
    int32_t rightDelta = encoder_getDelta(RIGHT);
    
    // Change signs
    
    if(sign == 1) // if sign is set, robot turns CCW (left backward, right forward)
    {
        leftPath = -leftPath;
        leftDelta = -leftDelta;
    }
    else // sign is unset, robot turns CW (left forward, right backward)
    {
        rightPath = -rightPath;
        rightDelta = -rightDelta;
    }

    int32_t midDelta = (leftDelta + rightDelta) / 2;
    int32_t midPath = (leftPath + rightPath) / 2;

    int32_t acceleration = midDelta - lastDelta;
    lastDelta = midDelta;

    // Stage 2
    // Start or continue robot to rotate
    // Check if robot still accelerating or moving
    // If it is true, check if we need to brake engines
    if(moveMode == 5) // accelerating
    {
        // 0. Engines direction set by sign var
        _mdir = sign;

        // 1. Check if we are not at maximum PWM
        if(_movePWM < _destPWM) // we have a range to accelerate
        {
            _movePWM += _moveAcc;
        }
        else
        {
            _movePWM = _destPWM;
        }

        // 2. Check if we reached MinBrakeDelta speed
        if(flag_GetMinBrake == 0 && midDelta >= MinBrakeDelta) // first stage - reach this speed
        {
            flag_GetMinBrake = 1;
            MinBrakePath = midPath;
            numMeasures = 1;
        }
        else if(flag_GetMinBrake == 1 && acceleration <= 0 && _movePWM == _destPWM) // second stage - end of acceleration
        {
            flag_GetMinBrake = 2;
            MinBrakePath = midPath - MinBrakePath;
            deltaSpeed = MinBrakePath / numMeasures;
        }

        if(flag_GetMinBrake == 1) numMeasures++;

        // 3. Check if we have MinBrakePath left at the path or
        // we still accelerating but reached middle of the way
        if((flag_GetMinBrake == 2 && (_destPath - midPath) <= MinBrakePath) || (flag_GetMinBrake != 2 && (_destPath - midPath) <= midPath))
        {
            moveMode = 4;
            rqSpeed = midDelta - 2 * rqSpeed;
            if(flag_GetMinBrake != 2) deltaSpeed = midPath / numMeasures;
        }

    }

    // Stage 3. Braking subprogram
    // If it is time to brake engines, we should decrease PWM while robot is still
    // can move (in final, we should have
    else if(moveMode == 4)// stopping
    {
        // 0. Engines direction is still set by sign
        _mdir = sign;
        // We have pre-defined value - minimal speed. This value means than
        // at this encoder speed robot should move and make e-brake (with 
        // brakepath near 2 mm
        //
        // This value is get as a result of calibration process
        // which just change the value and check brakepath
        
        // Try to save encoder speed at defined value

        // Required speed is decreasing every tick
        rqSpeed -= deltaSpeed;
        if(rqSpeed < MinBrakeDelta) rqSpeed = MinBrakeDelta;
        _movePWM = _movePWM - 2 * (midDelta - rqSpeed); // it's like P-algo to save speed

        float angleError = rqAngle - getAngle(); // if we have overrun, error will be greater than 0

        if((sign == 1 && angleError <= 0) || (sign == 0 && angleError >= 0)) // we get destination point!
        {
            moveMode = 3; // launch correcter
            BrakeStart = midPath;
            _movePWM = 0;
            _move_stay(); // stop right now, I said!
            midPath = 0;
            flag_GetMinBrake = 0;
            return;
        }
    }

    else // moveMode == 3 - correcting angle 
    {
        rqSpeed = 1; // absolute minimal speed
        _movePWM = _movePWM - 2 * (midDelta - rqSpeed);

        // Get error in position
        float angleError = rqAngle - getAngle(); // if we have overrun, error will be greater than 0
        
        // Set direction of the engines if required
        if(!flag_CorrectorDirSet)
        {
            // Get error sign
            if(angleError > 0) _mdir = 1; // 1 is greater sign
            else _mdir = 0;

            // Compare it with main sign, set direction and flag for not to
            // Robot need to rotate CCW (_mdir = 1), if it rotated CCW before, but didn't reached target angle
            // (sign == 1, err == 0), or it rotated CW and did an overrun (sign == 0, err == 1)
            // In another situations it need to rotate CW (_mdir = 0)
            
            // Set flag for not to recalculate this value
            flag_CorrectorDirSet = 1;
        }

        if(_mdir == 1) // moving CCW
        {
            // At CCW rotation, we check for equivalence or overrun
            //
            // Also, it should be better to redo correction if overrun
            // is so large (in TODO)
            if(angleError <= 0)
            {
                _movePWM = 0;
                _move_stay();
                midPath = 0;
                moveMode = 0;
                flag_CorrectorDirSet = 0;
                _mdir = 0;
                return;
            }
        }
        else // moving CW
        {
            if(angleError >= 0)
            {
                _movePWM = 0;
                _move_stay();
                midPath = 0;
                moveMode = 0;
                flag_CorrectorDirSet = 0;
                _mdir = 0;
                return;
            }
        }
    }

    // Stabilisation by encoders path
    int32_t error = pid_calculateLinError(rightPath, leftPath);

    // Stage 4
    // Updating PID data (PWM overload calculated in this function)
    pid_update(error, _movePWM, &leftPWM, &rightPWM);

    
    // Last stage
    // PID algo thinks than robot is moving forward by both wheels, so
    // we need to change PWM values signs to make robot chassis rotating
    if(_mdir == 1) // CCW (left turns backward)
    {
        leftPWM = -leftPWM;
    }
    else // CW (right turns backward)
    {
        rightPWM = -rightPWM;
    }

    chassis_write(leftPWM, rightPWM);
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
    _destPath = (int32_t) (getChassisRadius() * dAngle);
    if(_destPath < 0) _destPath = -_destPath;
    _destPWM = pwm;
    _moveAcc = acceleration;

    if(dAngle > 0) sign = 1;
    else sign = 0;

    rqAngle = getAngle() + dAngle;
    
    // 2. Run algo in background
    moveMode = 5;
}

/**
 * Emergency stop
 */
void move_stop(void)
{

}

int32_t move_getMidAcc(void)
{
    return _midAcc;
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
    return moveMode;
}
