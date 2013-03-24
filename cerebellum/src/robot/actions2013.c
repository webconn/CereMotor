#include <robots/actions2013.h>

servo _bigpaw, _smallpaw, _elevator;
sensor_t * _limiter_h, * _limiter_l;

void actions_init(servo bpaw, servo spaw, servo elev, sensor_t * limit_h, sensor_t * limit_l)
{
    _bigpaw = bpaw;
    _smallpaw = spaw;
    _elevator = elev;
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
            servo_write(_bigpaw, 430);
        }
        else if(state == OPEN)
        {
            servo_write(_bigpaw, 1200);
        }
        else // state == BLOWED
        {
            servo_write(_bigpaw, 980);
        }
    }
}

void elevator_move(uint16_t state)
{
    if(state == UP)
    {
        servo_write(_elevator, 700); // rotate servo to top
        while(!sensor_read(_limiter_h));
        servo_write(_elevator, 889); // stop elevator
    }
    else // state == DOWN
    {
        servo_write(_elevator, 920); // rotate servo to down
        while(!sensor_read(_limiter_l));
        servo_write(_elevator, 889); // stop
    }
}
