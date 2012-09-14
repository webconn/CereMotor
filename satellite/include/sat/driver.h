#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>

/**
 * Type defines
 */

#ifndef DIR_INV
    #define FORWARD 1
    #define BACKWARD 2
    #define driver_get_dir() ((PINB >> 2) & 3)
#else
    #define FORWARD 2
    #define BACKWARD 1
    #define driver_get_dir() (~(PINB >> 2) & 3)
#endif

#define driver_get_speed() (OCR1B)

/**
 * Main functions
 *
 * Note: inline functions will not be exported to RPC
 */

void driver_init(void); // init I/O and timers

void driver_set_dir(uint8_t dir); // set moving directory
void driver_set_speed(uint16_t speed); // set moving speed
void driver_stop(void); // stop the motor

#endif
