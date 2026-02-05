/*
 * dht11.c
 *
 *  Created on: Jan 27, 2026
 *      Author: hoang
 */


#include "dht11.h"

/* ========= Bạn phải set timer để chạy 1MHz (1 tick = 1us)
   Ví dụ với STM32F103: TIM2 Prescaler = (SystemCoreClock/1000000)-1
   Period đủ lớn (>= 20000) để đếm vài ms.
*/

static void DHT11_SetPinOutput(DHT11_HandleTypeDef *dht) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dht->GPIOx, &GPIO_InitStruct);
}

static void DHT11_SetPinInput(DHT11_HandleTypeDef *dht) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; // DHT11 thường dùng điện trở kéo lên ngoài 4.7k~10k
    HAL_GPIO_Init(dht->GPIOx, &GPIO_InitStruct);
}

static inline void DHT11_DelayUs(DHT11_HandleTypeDef *dht, uint16_t us) {
    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (__HAL_TIM_GET_COUNTER(dht->htim) < us) {;}
}

// đợi pin về mức level trong timeout_us, return 0 OK, -1 timeout
static int DHT11_WaitForLevel(DHT11_HandleTypeDef *dht, GPIO_PinState level, uint32_t timeout_us) {
    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) != level) {
        if (__HAL_TIM_GET_COUNTER(dht->htim) > timeout_us) return -1;
    }
    return 0;
}

// đo độ rộng xung mức HIGH (us) với timeout, trả về -1 nếu lỗi
static int DHT11_MeasureHighUs(DHT11_HandleTypeDef *dht, uint32_t timeout_us) {
    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(dht->htim) > timeout_us) return -1;
    }
    return (int)__HAL_TIM_GET_COUNTER(dht->htim);
}

void DHT11_Init(DHT11_HandleTypeDef *dht) {
    // pin mặc định để input (nhàn rỗi mức HIGH do kéo lên)
    DHT11_SetPinInput(dht);
    // đảm bảo timer đang chạy
    HAL_TIM_Base_Start(dht->htim);
}

int DHT11_Read(DHT11_HandleTypeDef *dht, DHT11_Data_t *out) {
    uint8_t data[5] = {0};

    // 1) Start signal: kéo LOW >= 18ms
    DHT11_SetPinOutput(dht);
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(18);

    // 2) thả lên HIGH 20~40us rồi chuyển sang input
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_SET);
    DHT11_DelayUs(dht, 30);
    DHT11_SetPinInput(dht);

    // 3) DHT11 response: LOW ~80us, HIGH ~80us
    if (DHT11_WaitForLevel(dht, GPIO_PIN_RESET, 100) < 0) return -1;
    if (DHT11_WaitForLevel(dht, GPIO_PIN_SET,   100) < 0) return -2;
    if (DHT11_WaitForLevel(dht, GPIO_PIN_RESET, 100) < 0) return -3;

    // 4) Read 40 bits
    for (int i = 0; i < 40; i++) {
        // mỗi bit bắt đầu bằng LOW ~50us
        if (DHT11_WaitForLevel(dht, GPIO_PIN_SET, 80) < 0) return -4;

        // sau đó là HIGH: ~26-28us (bit 0) hoặc ~70us (bit 1)
        int high_us = DHT11_MeasureHighUs(dht, 120);
        if (high_us < 0) return -5;

        uint8_t bit = (high_us > 45) ? 1 : 0; // ngưỡng 45us khá an toàn
        data[i / 8] <<= 1;
        data[i / 8] |= bit;
    }

    // 5) checksum
    uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (sum != data[4]) return -6;

    if (out) {
        out->humidity_int     = data[0];
        out->humidity_dec     = data[1];
        out->temperature_int  = data[2];
        out->temperature_dec  = data[3];
        out->checksum         = data[4];
    }

    return 0;
}
