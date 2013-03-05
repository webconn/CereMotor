#include <cerebellum/deltacoords.h>
#include <cerebellum/robot.h>

struct _coords{
    float angle; // in radians
    int32_t x;
    int32_t y;
} _current_coords;

int32_t getX(void)
{
    return _current_coords.x;
}

int32_t getY(void)
{
    return _current_coords.y;
}

float getAngle(void)
{
    return _current_coords.angle;
}

void updateX(int32_t new_x)
{
    _current_coords.x = new_x;
}

void updateY(int32_t new_y)
{
    _current_coords.y = new_y;
}

void updateAngle(float new_angle)
{
    _current_coords.angle = new_angle;
}

int32_t getCentralSpeed(int32_t speed1, int32_t speed2)
{
    // Central speed (module) is just arithmetic mean of
    // two speeds
    // I dont think than we need to set fixed point for calculation (in case we will have
    // half part), so, let's just sum it and move binary right (division by 2)
    return (speed1 + speed2) / 2;
}

float getDeltaAngle(int32_t speed1, int32_t speed2)
{
    // Sure, I'm afraid of floating point computing on MCU, but
    // we have no alternatives (hmm.. only to reverce this and 
    // to create a large table of values of trigonometry :))
    return ((float) (speed1 - speed2) / ((float)2*getChassisRadius()));
}

void updateCoords(int32_t speed1, int32_t speed2)
{
    // Stage 1. Get central speed
    int32_t centralSpeed = getCentralSpeed(speed1, speed2);

    // Stage 2. Get delta angle
    float deltaAngle = getDeltaAngle(speed1, speed2);
    

    // Stage 3. Sum deltaAngle with old data of angle
    _current_coords.angle += deltaAngle;
    
    // Stage 4. Calculating deltas
    //
    // Hmm... Here we should think, which operation is faster:
    // sin() or cos(), and use the fastest one.
    // Another side of the triangle should be found
    // using Pythagorean theorem (emm..)
    //
    // TODO: Fix my brain about signs in deltas
    
    int32_t deltaX = (int32_t) ((float) centralSpeed * cos(_current_coords.angle));
    int32_t deltaY = (int32_t) ((float) centralSpeed * sin(_current_coords.angle));

    // Stage 5. Sum deltas with collected values
    _current_coords.x += deltaX;
    _current_coords.y += deltaY;

}
