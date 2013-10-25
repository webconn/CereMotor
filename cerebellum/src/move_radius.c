#include <cerebellum/move_radius.h>

#define exp 0.000001
#define is_zero(a) (((a) < exp) && ((a) > -exp))


static int32_t raduis = 0;
static float angle_offset = 0;

void move_setRadius(int32_t r)
{
    radius = r;
}

void move_initRadius(void)
{
    // 1. Clear encoders values
    encoder_reset(LEFT);
    encoder_reset(RIGHT);

    // 2. Set angle offset
    angle_offset = coords_getAngle();
}

int32_t move_pathErrorCalculator_radius(void)
{
    int32_t enc_left = encoder_getPath(LEFT);
    int32_t enc_right = encoder_getPath(RIGHT);

    float angle = coords_getAngle() - angle_offset;

    if(is_zero(angle))
        return 0;
    else
        return (enc_left + enc_right) / 2 / angle - radius; // TODO: here some problems could happen; check type conversions
}
