#include <robots/actions2013.h>

servo _bigpaw, _smallpaw, _elevator, _grip_l, _grip_r;
sensor_t * _limiter_h, * _limiter_l;

volatile int elevator_state = DOWN;

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

int elevator_moveCareful(uint16_t state)
{
    if(state == UP && elevator_state == DOWN)
    {
        elevator_state = UP;
        servo_write(_elevator, 660); // rotate servo to top
        while(!sensor_read(_limiter_h));
        servo_write(_elevator, 870); // stop elevator
    }
    else if(elevator_state == UP || elevator_state == FAIL) // state == DOWN
    {
        // Here is something interesting
        // We should measure a time to down elevator
        // If it is locked (timer overflow), return fail statement
        servo_write(_elevator, 940); // rotate servo to bottom
        extern void _delay_ms(uint32_t t);
        _delay_ms(100);

        uint32_t time = 0, limit = CONFIG_ELEVATOR_FAIL_TIME;

        if(elevator_state == FAIL)
            limit = limit / 2;

        while(!sensor_read(_limiter_l))
        {
            _delay_ms(20);
            time++;
            if(time >= limit || sensor_read(_limiter_h)) // if time is over or elevator moved up
            {
                elevator_state = FAIL;
                servo_write(_elevator, 660);
                _delay_ms(200);
                servo_write(_elevator, 894); // stop elevator
                return 0; // fail
            }
        }
        servo_write(_elevator, 894); // lock servo engine
        elevator_state = DOWN;
    }
    return 1;
}

void elevator_move(uint16_t state)
{
    if(state == UP && (elevator_state == DOWN || elevator_state == FAIL))
    {
        elevator_state = UP;
        servo_write(_elevator, 700); // rotate servo to top
        if(elevator_state == FAIL)
        {
            if(!sensor_read(_limiter_h))
            {
                while(!sensor_read(_limiter_h));
            }
            else
            {
                while(sensor_read(_limiter_h));
            }
        }
        while(!sensor_read(_limiter_h));
        servo_write(_elevator, 870); // stop elevator
    }
    else if(elevator_state == UP || elevator_state == FAIL) // state == DOWN
    {
        elevator_state = DOWN;
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

void take_glass(uint16_t side, uint16_t elevator_state)
{
    grip_set(side, UNHOLD);

    extern void _delay_ms(uint32_t time);
    _delay_ms(200);
    int32_t path = 0, i = 0, result = 0;

    //float angle = coords_getAngle();
    result = elevator_moveCareful(DOWN);

    while(!result && i < 2) // it should be a case when elevator is locked
    {
        //grip_set(side, HOLD);
        //_delay_ms(300);
        elevator_move(UP);
        path += mmToTicks(150);
        led_on(3);
        //move_line(3000, 30, mmToTicks(150));
        //while(move_isBusy());
        _delay_ms(300);
        //grip_set(side, UNHOLD);
        //_delay_ms(200);
        i++;
        result = elevator_moveCareful(DOWN);
    }

    grip_set(side, HOLD); // take a glass
    _delay_ms(300);

    //move_line(2000, 30, -path); // go to the start position
    if(elevator_state == UP)
        elevator_move(UP);
    //while(move_isBusy());

    if(path > 0)
    {
       // move_rotateAbsolute(2000, 30, angle);
        //while(move_isBusy());
    }

    led_off(3);
}
