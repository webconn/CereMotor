#include <robots/actions2013.h>

servo _bigpaw, _smallpaw, _elevator, _grip_l, _grip_r;
sensor_t * _limiter_h, * _limiter_l;

void actions_init(servo bpaw, servo spaw, servo elev, servo gl, servo gr, sensor_t * limit_h, sensor_t * limit_l)
{
    _bigpaw = bpaw;
    _smallpaw = spaw;
    _elevator = elev;
    _grip_l = gl;
    _grip_r = gr;
    _limiter_h = limit_h;
    _limiter_l = limit_l;
}

void paw_move(uint16_t paw, uint16_t state)
{
    if(paw == SMALL)
    {
        if(state == CLOSE)
        {
            servo_write(_smallpaw, 400);
        }
        else if(state == OPEN)
        {
            servo_write(_smallpaw, 700);
        }
        else // state == BLOWED
        {
            servo_write(_smallpaw, 1050);
        }
    }
    else // paw == BIG
    {
        if(state == CLOSE)
        {
            servo_write(_bigpaw, 450);
        }
        else if(state == OPEN)
        {
            servo_write(_bigpaw, 1220);
        }
        else // state == BLOWED
        {
            servo_write(_bigpaw, 1000);
        }
    }
}

void elevator_move(uint16_t state)
{
    if(state == UP)
    {
        servo_write(_elevator, 700); // rotate servo to top
        while(!sensor_read(_limiter_h));
        servo_write(_elevator, 870); // stop elevator
    }
    else // state == DOWN
    {
        servo_write(_elevator, 925); // rotate servo to bottom
        while(!sensor_read(_limiter_l));
        servo_write(_elevator, 894); // lock servo engine
    }
}

void grip_set(uint16_t grip, uint16_t state)
{
    if(grip == LEFT)
    {
        if(state == UNHOLD)
        {
            servo_write(_grip_l, 1050);
        }
        else if(state == HOLD)
        {
            servo_write(_grip_l, 1200);
        }
        else // if state == OPEN
        {
            servo_write(_grip_l, 900);
        }
    }
    else // grip == RIGHT
    {
        if(state == UNHOLD)
        {
            servo_write(_grip_r, 630);
        }
        else if(state == HOLD)
        {
            servo_write(_grip_r, 530);
        }
        else // if state == OPEN
        {
            servo_write(_grip_r, 780);
        }
    }
}
