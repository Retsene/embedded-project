#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;        // 7-bit address (vd: 0x27 hoặc 0x3F)
    uint8_t backlight;   // 0 or 1
} LCD1602_I2C_t;

void LCD1602_Init(LCD1602_I2C_t *lcd, I2C_HandleTypeDef *hi2c, uint8_t addr7bit);
void LCD1602_Backlight(LCD1602_I2C_t *lcd, uint8_t on);
void LCD1602_Clear(LCD1602_I2C_t *lcd);
void LCD1602_SetCursor(LCD1602_I2C_t *lcd, uint8_t row, uint8_t col);
void LCD1602_Print(LCD1602_I2C_t *lcd, const char *s);
