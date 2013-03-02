#ifndef CEREBELLUM_MOVEMENT
#define CEREBELLUM_MOVEMENT

#include <stm32f10x.h>

// Functions
void move_tick(void);
void move_line(uint16_t pwm, uint16_t acceleration, uint32_t path);
int32_t move_getPWM(uint8_t val);
int move_isBusy(void);

#endif
