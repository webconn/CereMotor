#ifndef CEREBELLUM_CHASSIS_H
#define CEREBELLUM_CHASSIS_H

/**
 * @file cerebellum/chassis.h
 * @brief Headers for chassis driver interface
 * @author WebConn
 * @date 4 Jan 2013
 * @version 0.1
 */

#include <stm32f10x.h>

void chassis_init(void);
void chassis_write(int16_t left, int16_t right);
void chassis_break(uint16_t left, uint16_t right);
void chassis_set_dir(uint8_t engine, uint8_t direction);

#endif
