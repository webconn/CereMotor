#include <cerebellum/points.h>

point_t _point_currentPosition;
/*
point_t * point_getPosition(void)
{
    _point_currentPosition.x = getX();
    _point_currentPosition.y = getY();
    _point_currentPosition.angle = getAngle();

    return &_point_currentPosition;
}

void point_setPosition(point_t * position)
{
    updateX(position->x);
    updateY(position->y);
    updateAngle(position->angle);
}

void point_goto(point_t * destination)
{
    // TODO: get point movement macros
}

void point_trace(point_t * destination, int32_t radius)
{
    // 0. Get start position
    point_t * startPosition = point_getPosition();

    // 1. Calculate critical point position, according to radius and destination
    point_t criticalPoint;
    criticalPoint.x = destination->x - radius * cos(destination->angle);
    criticalPoint.y = destination->y - radius * sin(destination->angle);
    criticalPoint.angle = destination->angle;

    // 2. We need to know deltaAngle for route
    float deltaAngle = point_optimiseAngle(destination->angle, ZERO_TWO_PI) - point_optimiseAngle(startPosition->angle, ZERO_TWO_PI);

    // 3. Extract deltaAngle sign for easier comparison
    int32_t deltaAngleSign = 0;
    if(deltaAngle < 0)
    {
        deltaAngleSign = 1;
        deltaAngle = -deltaAngle;
    }

    // 4. Compare deltaAngle with defined segments of circle
    if(deltaAngle == 0) // zero position
    {
        // In the case of zero position, we should have two situations
        // First is when destination is in front of us (the most easier)
        
        // Second is when destination is behind us
        else
        {

        }
    }
    else if(deltaAngle < 3.141593 / 2) // 1st quarter
    {
        
    }
    else if(deltaAngle == 3.141593 / 2) // 90 degrees
    {

    }
    else if(deltaAngle < 3.141593) // 2nd quarter
    {

    }
    else if(deltaAngle == 3.141593) // 180 degrees
    {

    }
    else if(deltaAngle < 3.141593 * 3 / 2) // 3rd quarter
    {

    }
    else if(deltaAngle == 3.141593 * 3 / 2) // 270 degrees
    {

    }
    else // if(deltaAngle < 3.141593 * 2) // 4th quarter
    {

    }
}
*/
float point_optimiseAngle(float angle, uint32_t segment)
{
    // Recalculate angle for defined segment
    if(segment == MINUS_PI_PLUS_PI)
    {
        while(angle > 3.141593)
        {
            angle -= 2 * 3.141593;
        }
        while(angle <= -3.141593)
        {
            angle += 2 * 3.141593;
        }
    }
    else if(segment == ZERO_TWO_PI)
    {
        while(angle > 2 * 3.141593)
        {
            angle -= 2 * 3.141593;
        }
        while(angle <= 0)
        {
            angle += 2 * 3.141593;
        }
    }

    return angle;
}
