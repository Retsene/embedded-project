/*
 * time.delay.c
 *
 *  Created on: Jan 27, 2026
 *      Author: hoang
 */

#include "time_delay.h"

static TIM_HandleTypeDef *g_htim = NULL;

void TIM2_Delay_Init(TIM_HandleTypeDef *htim)
{
    g_htim = htim;
    __HAL_TIM_SET_COUNTER(g_htim, 0);
}

uint32_t micros(void)
{
    return __HAL_TIM_GET_COUNTER(g_htim); // 1 tick = 1us (vì PSC đã set)
}

void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) { }
}

