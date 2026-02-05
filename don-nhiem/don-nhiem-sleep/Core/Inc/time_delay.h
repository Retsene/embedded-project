/*
 * time_delay.h
 *
 *  Created on: Jan 27, 2026
 *      Author: hoang
 */

#ifndef INC_TIME_DELAY_H_
#define INC_TIME_DELAY_H_
#pragma once
#include "stm32f1xx_hal.h"

void TIM2_Delay_Init(TIM_HandleTypeDef *htim);
uint32_t micros(void);
void delay_us(uint32_t us);


#endif /* INC_TIME_DELAY_H_ */
