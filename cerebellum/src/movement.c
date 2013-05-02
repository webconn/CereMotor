#include <cerebellum/movement.h>
#include <stdio.h>
#include <cerebellum/points.h>

// MinBrakeDelta is a calibrated value
int32_t MinBrakeDelta = 0;
int32_t BrakeStart = 0;
volatile uint8_t relativeDirection = 0; // direction to monitor via ODetect
uint8_t _move_save = 1, stabMode = 0;

int moveMode = 0;

extern void _delay_ms(uint32_t ms);
void _move_clear(void);

void move_saveSwitch(FunctionalState state)
{
    if(state == DISABLE)
    {
        _move_save = 0;
    }
    else
    {
        _move_save = 1;
    }
}

int32_t __errno; // for sqrt()
void move_toPoint(int32_t x, int32_t y, int32_t pwm, int stab)
{
    // Stage 1. Get current coordinates
    int32_t cx, cy;
    cx = getX();
    cy = getY();

    // Stage 2. Get current angle
    float cangle = getAngle();

    // Stage 3. Calculate path
    int32_t path = (int32_t) sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
    float angle = atan2((float) (y-cy), (float) (x-cx));

    // Stage 4. Rotate robot to the new angle
    move_rotate(pwm / 2, 30, angle - cangle);
    while(moveMode > 0)
        _delay_ms(10);

    // Stage 5. Go to the path
    move_line(pwm, 15, path);
    while(moveMode > 0)
        _delay_ms(10);

    // Stage 6. Correct angle
    if(stab)
    {
        cangle = getAngle();
        move_rotate(pwm / 2, 30, angle - cangle);
        while(moveMode > 0)
            _delay_ms(10);
    }
}

void move_toPointBackward(int32_t x, int32_t y, int32_t pwm, int stab)
{
    // Stage 1. Get current coordinates
    int32_t cx, cy;
    cx = getX();
    cy = getY();

    // Stage 2. Get current angle
    float cangle = getAngle();

    // Stage 3. Calculate path
    int32_t path = (int32_t) sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
    float angle = atan2((float) (cy-y), (float) (cx-x));

    // Stage 4. Rotate robot to the new angle
    move_rotate(pwm / 2, 30, angle - cangle);
    while(moveMode > 0)
        _delay_ms(10);

    // Stage 5. Go to the path
    move_line(pwm, 15, -path);
    while(moveMode > 0)
        _delay_ms(10);

    // Stage 6. Correct angle
    if(stab)
    {
        cangle = getAngle();
        move_rotate(pwm / 2, 30, angle - cangle);
        while(moveMode > 0)
            _delay_ms(10);
    }
}

void move_setMinBrakeDelta(int32_t value)
{
    MinBrakeDelta = value;
}

int32_t move_getBrakePath(void)
{
    return BrakeStart;
}

inline void _move_stay(void);
inline void _move_line(void);
inline void _move_rotate(void);
inline void _move_wall(void);

sensor_t * _limiter_left, * _limiter_right, * _rf_front, * _rf_rear;

void move_initLimiters(sensor_t * lim_l, sensor_t * lim_r)
{
    // Here we init barrier rangefinders
    // and limiter buttons
    _limiter_left = lim_l;
    _limiter_right = lim_r;
}

void move_initWallSensor(sensor_t * rf_front, sensor_t * rf_rear)
{
    _rf_front = rf_front;
    _rf_rear = rf_rear;
}

// Tick function - running freely in SysTick and updating configuration
void move_tick(void)
{
    if(moveMode == 0)
        _move_stay();
    else if(moveMode < 3)
        _move_line();
    else if(moveMode < 6)
        _move_rotate();
    else if(moveMode < 8)
        _move_wall();
}

// Stay tick function
inline void _move_stay(void)
{
    // To save robot's coordinates, we should block motors
    // and save this state
    chassis_break(CONFIG_PWM_ACCURACY, CONFIG_PWM_ACCURACY); // PWM at maximum

    if(_move_save)
    {
        int32_t leftPath = encoder_getPath(1);
        int32_t rightPath = encoder_getPath(0);

        chassis_write(-16 * leftPath, -16 * rightPath);
    }
}

int32_t _movePWM, _moveAcc;
int32_t _destPWM;
int32_t _accPath = 0, _destPath;
int32_t lastSpeed = 0;
int32_t _midAcc = 0;
int32_t sign = 0;
int32_t _numMeasures = 0;
int32_t _moveDirection = 0; // 0 for forward, 1 for backward
float startAngle = 0;

int32_t leftPWM, rightPWM;

inline void _move_line(void)
{
    // Stabilisation algo depends on user selection - defined

    // 1. Get paths by both encoders
    int32_t leftPath = encoder_getPath(1);
    int32_t rightPath = encoder_getPath(0);

    int32_t leftSpeed = encoder_getDelta(1);
    int32_t rightSpeed = encoder_getDelta(0);
    
    float angle = getAngle();

    if(_moveDirection)
    {
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
        leftPath = -leftPath;
        rightPath = -rightPath;
    }

    int32_t aripPath = (leftPath + rightPath) / 2; // sum and divide by 2

    int32_t acceleration = ((leftSpeed + rightSpeed) / 2) - lastSpeed;

    lastSpeed = (leftSpeed + rightSpeed) / 2;

    // Speed edjes: when starting and when stopping
    if(moveMode == 2) // normal operation (no brakes)
    {
        // 1. Calculate max acceleration
        if(_midAcc < acceleration)
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
        if((_destPath - aripPath) <= mmToTicks(50) || (_accPath == 0 && (_destPath - aripPath) <= aripPath))
        {
            moveMode = 1;
        }
        
        // Check acceleration: if accelerated, take the measure
        if(acceleration <= 0 && !_accPath && _movePWM == _destPWM)
        {
            _accPath = aripPath;
        }
    }
    else 
    {
        // At this stage we need to control encoders speed
        if(leftSpeed > MinBrakeDelta && rightSpeed > MinBrakeDelta && acceleration >= -_midAcc) // speed down while we should move
            _movePWM -= _moveAcc;
        else if(leftSpeed < MinBrakeDelta || rightSpeed < MinBrakeDelta)
            _movePWM += _moveAcc;
        
        // If we reached end, stop engines and shut algo down
        if(aripPath >= _destPath)
        {
            _movePWM = 0;
            _accPath = 0;
            _midAcc = 0;
            move_stop();
            return;
        }
    }

    // Stabilisation by encoders path
    int32_t error = 0;

    if(stabMode == 1)
        error = pid_calculateLinError(rightPath, leftPath);
    else
    {
        angle -= startAngle;

        if((angle > 0 && angle < 0.01) || (angle < 0 && angle > -0.01))
        {
            if(_movePWM <= 1000)
                error = (int32_t) 22000 * angle;
            else if(_movePWM <= 2000)
                error = (int32_t) 20000 * angle;
            else if(_movePWM <= 2500)
                error = (int32_t) 36000 * angle;
            else
                error = (int32_t) 40000 * angle;
        }
        else
        {
            if(_movePWM <= 1000)
                error = (int32_t) 24000 * angle;
            else if(_movePWM <= 2000)
                error = (int32_t) 24000 * angle;
            else if(_movePWM <= 2500)
                error = (int32_t) 40000 * angle;
            else
                error = (int32_t) 44000 * angle;
    }

    if(_moveDirection)
        error = -error;
    }
    // 3. Updating PID data (PWM overload calculated in this function)
    pid_update(error, _movePWM, &leftPWM, &rightPWM);

    // 4. Write data to chassis
    if(_moveDirection)
        chassis_write(-leftPWM, -rightPWM);
    else
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
            pid_reset();
            return;
        }
    }

    else // moveMode == 3 - correcting angle 
    {
        rqSpeed = MinBrakeDelta; // absolute minimal speed
        _movePWM = _movePWM - 2 * (midDelta - rqSpeed);

        // Get error in position
        float angleError = rqAngle - getAngle(); // if we have overrun, error will be greater than 0
        
        // Set direction of the engines if required
        if(!flag_CorrectorDirSet)
        {
            // Get error sign
            if(angleError > 0) _mdir = 1; // 1 is greater sign
            else _mdir = 0;

            sign = _mdir;

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
                move_stop();
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
                move_stop();
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

/*
 * Move near the wall (by using barrier rangefinders)
 */

int32_t integral = 0;
int32_t oldState = 0;
int32_t pulse = 0;
int32_t _oldState = 0;
int32_t _speed = 0;

void _move_wall(void)
{
    // 1. Get paths by both encoders
    int32_t leftPath = encoder_getPath(1);
    int32_t rightPath = encoder_getPath(0);

    int32_t leftSpeed = encoder_getDelta(1);
    int32_t rightSpeed = encoder_getDelta(0);

    if(sign)
    {
        leftPath = -leftPath;
        rightPath = -rightPath;
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
    }

    int32_t aripPath = (leftPath + rightPath) / 2; // sum and divide by 2

    /*if(moveMode == 7) // normal operation (no brakes)
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
            moveMode = 6;
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
*/
        
        // If we reached end, stop engines and shut algo down
        if(aripPath >= _destPath)
        {
            move_stop(); // stop engines
            _movePWM = 0;
            _accPath = 0;
            _midAcc = 0;
            _move_stay(); // stop right now, I said!
            return;
        }
    //}

    // Stabilisation by barrier rangefinders
    // We think that ON-state of rangefinder is
    // when range is less than barrier (we are so
    // close to wall)
    // rf1 is front, rf2 is rear

    // 1. Read sensors values and calculate errors
    int32_t error = 0;
    if(!sign)
        error = 1400 - sensor_read(_rf_rear);
    else
        error = 2800 - sensor_read(_rf_front);

    // If error is positive, wall is far
    // If error is negative, wall is too close

    // Now check out the speed
    if(leftSpeed + rightSpeed > 2 * _speed)
        _movePWM -= _moveAcc;
    else
        _movePWM += _moveAcc;

    // Multiply errors by coefficient

    pid_update(error, _movePWM, &rightPWM, &leftPWM);

    // Calculate left and right values
    if(sign) // move backward
        chassis_write(-leftPWM, -rightPWM);
    else
        chassis_write(leftPWM, rightPWM);
    

    // 1. Collect data from rangefinders
    /*int32_t state;
    if(sign)
        state = ((sensor_read(_rf_rear) > 0) << 1)|((sensor_read(_rf_front) > 0));
    else
        state = ((sensor_read(_rf_front) > 0) << 1)|((sensor_read(_rf_rear) > 0));

    // 2. Analyse the state
    if(state >= 2) // go away from wall
    {
        state++;
    }
    state -= 2; // now we have coefficient for movement direction and intensivity

    
    if(state != _oldState)
        integral = 0;
    _oldState = state;

    integral += state;

    if(state > 0)
    {
        leftPWM = _movePWM;
        rightPWM =  -integral;
    }
    else
    {
        state = -state;
        leftPWM = -integral;
        rightPWM = _movePWM;
    }*/

   /* if(sign)
        chassis_write(-leftPWM, -rightPWM);
    else
        chassis_write(leftPWM, rightPWM);*/
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
void move_line(int32_t pwm, int32_t acceleration, int32_t path)
{
    if(path == 0)
        return; // no movement is required

    // To go by line, we need to run PID algorithm
    // stabilising paths or angle
    
    // 0. Clear encoder data
    encoder_reset(0);
    encoder_reset(1);
    _move_clear();

    // 1. Set data
    if(path < 0)
    {
        path = -path;
        _moveDirection = 1;
        relativeDirection = DIR_BACKWARD;
    }
    else
    {
        _moveDirection = 0;
        relativeDirection = DIR_FORWARD;
    }

    if(path < mmToTicks(150))
        stabMode = 0;
    else
        stabMode = 0;

    _destPWM = pwm;
    _moveAcc = acceleration;
    _destPath = path;
    startAngle = getAngle();

    // 2. Reset PID data
    pid_reset();

    // 3. Run algo in background
    moveMode = 2;
}

/**
 * Rotate initializer
 */
void move_rotate(int32_t pwm, int32_t acceleration, float dAngle)
{
    if((dAngle >= 0 && dAngle < 0.001) || (dAngle < 0 && dAngle > -0.001))
        return; // no rotation is needed
    // 0. Clear encoders and PWM
    pid_reset();
    encoder_reset(0);
    encoder_reset(1);
    _move_clear();

    // 1. Set data
    _destPath = (int32_t) (getChassisRadius() * dAngle);
    if(_destPath < 0) _destPath = -_destPath;
    _destPWM = pwm;
    _moveAcc = acceleration;

    if(dAngle > 0){
        sign = 1;
    }
    else sign = 0;

    relativeDirection = DIR_LEFT;

    rqAngle = getAngle() + dAngle;
    
    // 2. Run algo in background
    moveMode = 5;
}

/**
 * Absolute rotate initializer
 */
void move_rotateAbsolute(int32_t pwm, int32_t acceleration, float dAngle)
{
    // 0. Clear encoders and PWM
    pid_reset();
    encoder_reset(0);
    encoder_reset(1);
    _move_clear();

    // 1. Set data
    _destPath = (int32_t) (getChassisRadius() * (dAngle - getAngle()));
    sign = 1;
    if(_destPath < 0)
    {
        _destPath = -_destPath;
        sign = 0;
    }
    _destPWM = pwm;
    _moveAcc = acceleration;

    relativeDirection = DIR_LEFT;

    rqAngle = dAngle;
    
    // 2. Run algo in background
    moveMode = 5;
}

/**
 * Move by wall initializer
 */
void move_wall(int32_t speed, int32_t acceleration, int32_t path)
{
    if(path == 0)
        return; // no movement is required
    // 0. Clear encoder data
    encoder_reset(0);
    encoder_reset(1);
    _move_clear();
    pid_reset();

    // 1. Set data
    _moveAcc = acceleration;
    _speed = speed;

    sign = 0;
    if(path < 0)
    {
        path = -path;
        sign = 1;
        relativeDirection = DIR_BACKWARD;
    }
    else
    {
        relativeDirection = DIR_FORWARD;
    }

    _destPath = path;

    // 2. Run algo in background
    moveMode = 7;
}


/**
 * Clear movement parametres
 */
void _move_clear(void)
{
    flag_GetMinBrake = 0;
    flag_CorrectorDirSet = 0;
    MinBrakePath = 0;
    lastDelta = 0;
    numMeasures = 0;
    deltaSpeed = 0;
    rqSpeed = 0;
    rqAngle = 0;
    _mdir = 0;
    _movePWM = 0;
    _moveAcc = 0;
    _destPWM = 0;
    _accPath = 0;
    _destPath = 0;
    lastSpeed = 0;
    _midAcc = 0;
    sign = 0;
    _numMeasures = 0;
    relativeDirection = 0;
}

/**
 * Emergency stop
 */
void move_stop(void)
{
    // Just stop the engines and clear moveMode
    encoder_reset(0);
    encoder_reset(1);
    moveMode = 0;
    _move_stay();
}

int32_t move_getMidAcc(void)
{
    return _midAcc;
}

uint8_t move_getRelativeDirection(void)
{
    return relativeDirection;
}

/**
 * Moving pause
 */

struct {
    uint32_t moveMode;
    int32_t leftPath;
    int32_t rightPath;
    pidConfig pid;   
} _move_dump;

volatile uint8_t _move_paused = 0;

void move_pause(void)
{
    if(!_move_paused)
    {
        _move_save = 1;
        _move_paused = 1;
        // 1. Collect required data
        _move_dump.moveMode = moveMode;
        _move_dump.leftPath = encoder_getPath(1);
        _move_dump.rightPath = encoder_getPath(0);
        _move_dump.pid = pid_dumpData();

        move_stop();
    }
}

/**
 * Moving continue
 */

void move_continue(void)
{
    // Turn data back
    moveMode = _move_dump.moveMode;
    encoder_setPath(1, _move_dump.leftPath);
    encoder_setPath(0, _move_dump.rightPath);
    _move_paused = 0;
    _move_save = 0;
    pid_config(&_move_dump.pid);
}

/**
 * Refresh angle procedure
 * Robot moves to wall, and, when it
 * reaches wall, refresh the angle and stop engines
 */

void move_refreshAngle(void)
{
    odetect_switch(DISABLE);

    // 0. Select correcting angle
    float currentAngle = getAngle();
    //int sign = 1;

    while(currentAngle > 2 * PI)
    {
        currentAngle -= 2 * PI;
    }
    while(currentAngle < - 2 * PI)
    {
        currentAngle += 2 * PI;
    }

    if(currentAngle < 0)
    {
        currentAngle = -currentAngle;
    //    sign = -1;
    }

    // Select current angle
    float realAngle = 0;
    if(currentAngle >= PI / 4 && currentAngle <= 3 * PI / 4)
        realAngle = PI / 2;
    else if(currentAngle >= 3 * PI / 4 && currentAngle <= 5 * PI / 4)
        realAngle = PI;
    else if(currentAngle >= 5 * PI / 4 && currentAngle <= 7 * PI / 4)
        realAngle = 3 * PI / 2;
    // Else realAngle = 0 - ready
    
    // 1. Rotate closely to required angle for not to destroy something
    /*move_rotate(2000, 20, realAngle - sign * currentAngle);
    while(moveMode > 0)
    {
        _delay_ms(10);
    }*/

    // 1. Init movement by straight line to back with minimal speed
    move_line((CONFIG_PWM_ACCURACY / 2), 20, -mmToTicks(500));

    // 2. Wait while limiters are inactive
    while(moveMode > 0)
    {
        if(sensor_read(_limiter_left) || sensor_read(_limiter_right))
            break;
        if(moveMode == 1) // robot tries to stop
        {
            moveMode = 2; // make it continue moving
            _destPath -= mmToTicks(10);
        }
    }
    
    moveMode = 990; // wall clear
    chassis_write(-3000, -3000);

    while(!sensor_read(_limiter_left) || !sensor_read(_limiter_right));

    // Little delay
    extern void _delay_ms(uint32_t time);
    // 3. Now e-stop engines
    _delay_ms(500);
    move_stop();

    _delay_ms(300);

    // Update angle
    updateAngle(realAngle);
    odetect_switch(ENABLE);
} 

/**
 * Check movement system business
 */
int move_isBusy(void)
{
    if(_move_paused)
        return 1;
    else
        return moveMode;
}

void move_free(void)
{
    moveMode = 999; // no control
}
