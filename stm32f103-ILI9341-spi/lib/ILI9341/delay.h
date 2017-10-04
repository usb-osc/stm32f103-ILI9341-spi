#ifndef DELAY_H
#define DELAY_H

#include "stm32f10x.h"

void SysTick_Handler(void);
void delay(uint32_t time, uint32_t load);
void delay_us(uint32_t time);
void delay_ms(uint32_t time);

#endif	/* DELAY_H */
