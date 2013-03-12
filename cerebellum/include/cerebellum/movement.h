#ifndef CEREBELLUM_MOVEMENT
#define CEREBELLUM_MOVEMENT

#include <stm32f10x.h>

// Functions
void move_tick(void);
void move_line(uint32_t pwm, uint32_t acceleration, uint32_t path);
void move_rotate(uint32_t path, uint32_t acceleration, float angle);
int32_t move_getPWM(uint8_t val);
int move_isBusy(void);

void move_setMinBrakeDelta(int32_t value);
int32_t move_getBrakePath(void);

#endif
