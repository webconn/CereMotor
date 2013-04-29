#ifndef CEREBELLUM_ROBOT
#define CEREBELLUM_ROBOT

#include <robots/config.h>
#include <math.h>

/**
 * @addgroup Conversions
 */

#define mmToTicks(m) ((int32_t) ((m*CONFIG_ENC_RESOLUTION*CONFIG_PATH_RCOEFF)/(2*CONFIG_ENC_WHEEL_RADIUS*M_PI)))
#define degreesToRadians(d) ((float) (d * M_PI / 180))
#define getChassisRadius() mmToTicks(CONFIG_CHASSIS_RADIUS)

#endif
