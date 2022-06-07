/*
 * Timer_Delay.h
 *
 *  Created on: Jun 22, 2020
 *      Author: Khaled Magdy
 */

#ifndef TIMER_DELAY_H_
#define TIMER_DELAY_H_

#include "stm32l1xx_hal.h"


void TimerDelay_Init(void);
void delay_us(uint16_t au16_us);
void delay_ms(uint16_t au16_ms);

#endif /* TIMER_DELAY_H_ */
