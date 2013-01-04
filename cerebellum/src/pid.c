#include <cerebellum/pid.h>
#include <cerebellum/robot.h>

pidConfig _pid;

int32_t calculateLinError(int32_t speed1, int32_t speed2)
{
    // The linear error is just substraction of speeds
    return speed1-speed2;
}

int32_t calculateRadError(int32_t speed1, int32_t speed2, uint32_t radius)
{
    // Radial error is something more difficult
    // In fact, the dependence looks like this:
    // V1*(R-r) == V2*(R+r)
    // 
    // But this dependence IRL looks like this:
    // Va == Vb
    //
    // So, Va = V1*(R-r), Vb = V2*(R+r)
    //
    // Let's get V1 - V2
    uint32_t chassisRadius = getChassisRadius();
    return (speed1 / (int32_t) (radius - chassisRadius)) - (int32_t) (speed2 / (radius + chassisRadius));
}

int32_t getRequiredPWM(int32_t requiredSpeed)
{
    // TODO it.
    // We need to get relations between IRL speed and PWM wideness.
    //
    // Some cases to resolve this problem:
    // 1. Use linear dependence
    // 2. Collect data about current PWM and current speed and, so, calculate the speed
    //
    // It should be better to think about 2nd case, but now we will use 1st.
    //
    // There's something wrong, coefficient depends on frequency of collector timer
    // and on wheel radius and on encoder resolution etc.
    return 2*requiredSpeed; 
}

void updatePID(int32_t error, int32_t requiredPWM, int32_t * value1, int32_t * value2)
{
    // 1. Calculating integral
    _pid.i_mem += error;

    if(_pid.i_mem > _pid.i_max)
        _pid.i_mem = _pid.i_max;
    else if(_pid.i_mem < _pid.i_min)
        _pid.i_mem = _pid.i_min;

    int32_t integral = _pid.i_mem;

    // 2. Calculating derivative
    int32_t derivative = _pid.d_mem;
    _pid.d_mem = error;

    // 3. Calculating full sentence
    int32_t correction = (error * _pid.p_gain) + (integral / _pid.i_rgain) - (derivative / _pid.d_rgain);

    // 4. Return new data
    *value1 = requiredPWM - correction;
    *value2 = requiredPWM + correction;
    // TODO: if something goes wrong, just change signs :)
}

void resetPID(void)
{
    _pid.i_mem = 0;
    _pid.d_mem = 0;
}

void configPID(pidConfig * data)
{
    _pid.p_gain = data->p_gain;
    _pid.i_rgain = data->i_rgain;
    _pid.d_rgain = data->d_rgain;

    _pid.i_min = data->i_min;
    _pid.i_max = data->i_max;
}
