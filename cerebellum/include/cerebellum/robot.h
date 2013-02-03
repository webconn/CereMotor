#ifndef CEREBELLUM_ROBOT
#define CEREBELLUM_ROBOT

#include <robots/config.h>
#include <math.h>

/**
 * @addgroup Conversions
 */

#define mmToTicks(m) ((uint32_t) ((m*CONFIG_ENC_RESOLUTION)/(2*CONFIG_ENC_WHEEL_RADIUS*M_PI)))
#define getChassisRadius() mmToTicks(CONFIG_CHASSIS_RADIUS)

#endif
