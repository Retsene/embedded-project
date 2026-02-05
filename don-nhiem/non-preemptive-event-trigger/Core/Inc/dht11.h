/*
 * dht11.h
 *
 *  Created on: Jan 27, 2026
 *      Author: hoang
 */
#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    TIM_HandleTypeDef *htim;
} DHT11_HandleTypeDef;

typedef struct {
    uint8_t humidity_int;
    uint8_t humidity_dec;
    uint8_t temperature_int;
    uint8_t temperature_dec;
    uint8_t checksum;
} DHT11_Data_t;

void DHT11_Init(DHT11_HandleTypeDef *dht);
int  DHT11_Read(DHT11_HandleTypeDef *dht, DHT11_Data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* INC_DHT11_H_ */
