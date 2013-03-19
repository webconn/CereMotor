#ifndef CEREBELLUM_SNIPPETS_H
#define CEREBELLUM_SNIPPETS_H

#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

void _snippet_runGPIORCC(GPIO_TypeDef * gpio);
void _snippet_runTIMRCC(TIM_TypeDef * tim);

#endif
