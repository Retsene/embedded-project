#include "i2c_LCD.h"
#include "time_delay.h"

// PCF8574 mapping phổ biến:
// P0=RS, P1=RW, P2=EN, P3=Backlight, P4..P7 = D4..D7
#define LCD_RS   (1U<<0)
#define LCD_RW   (1U<<1)
#define LCD_EN   (1U<<2)
#define LCD_BL   (1U<<3)

static void pcf_write(LCD1602_I2C_t *lcd, uint8_t data)
{
    uint8_t d = data | (lcd->backlight ? LCD_BL : 0);
    // HAL expects 8-bit address => shift left 1
    HAL_I2C_Master_Transmit(lcd->hi2c, (uint16_t)(lcd->addr << 1), &d, 1, 50);
}

static void lcd_pulse_enable(LCD1602_I2C_t *lcd, uint8_t data)
{
    pcf_write(lcd, data | LCD_EN);
    delay_us(1);
    pcf_write(lcd, data & ~LCD_EN);
    delay_us(50);
}

static void lcd_write4(LCD1602_I2C_t *lcd, uint8_t nibble, uint8_t control)
{
    // nibble: high 4 bits contain data for D4..D7
    uint8_t data = (nibble & 0xF0) | control;
    pcf_write(lcd, data);
    lcd_pulse_enable(lcd, data);
}

static void lcd_send(LCD1602_I2C_t *lcd, uint8_t value, uint8_t mode_rs)
{
    uint8_t control = mode_rs ? LCD_RS : 0; // RW = 0 always
    lcd_write4(lcd, value & 0xF0, control);
    lcd_write4(lcd, (value << 4) & 0xF0, control);
}

static void lcd_cmd(LCD1602_I2C_t *lcd, uint8_t cmd)
{
    lcd_send(lcd, cmd, 0);
    if (cmd == 0x01 || cmd == 0x02) HAL_Delay(2); // clear/home need >1.5ms
}

static void lcd_data(LCD1602_I2C_t *lcd, uint8_t data)
{
    lcd_send(lcd, data, 1);
}

void LCD1602_Backlight(LCD1602_I2C_t *lcd, uint8_t on)
{
    lcd->backlight = on ? 1 : 0;
    pcf_write(lcd, 0); // refresh
}

void LCD1602_Init(LCD1602_I2C_t *lcd, I2C_HandleTypeDef *hi2c, uint8_t addr7bit)
{
    lcd->hi2c = hi2c;
    lcd->addr = addr7bit;
    lcd->backlight = 1;

    HAL_Delay(50); // wait LCD power up

    // init 4-bit mode sequence
    lcd_write4(lcd, 0x30, 0);
    HAL_Delay(5);
    lcd_write4(lcd, 0x30, 0);
    delay_us(150);
    lcd_write4(lcd, 0x30, 0);
    delay_us(150);
    lcd_write4(lcd, 0x20, 0); // 4-bit
    HAL_Delay(2);

    lcd_cmd(lcd, 0x28); // 4-bit, 2 line, 5x8
    lcd_cmd(lcd, 0x0C); // display on, cursor off
    lcd_cmd(lcd, 0x06); // entry mode
    lcd_cmd(lcd, 0x01); // clear
}

void LCD1602_Clear(LCD1602_I2C_t *lcd)
{
    lcd_cmd(lcd, 0x01);
}

void LCD1602_SetCursor(LCD1602_I2C_t *lcd, uint8_t row, uint8_t col)
{
    // row: 0/1
    uint8_t addr = (row == 0) ? (0x00 + col) : (0x40 + col);
    lcd_cmd(lcd, 0x80 | addr);
}

void LCD1602_Print(LCD1602_I2C_t *lcd, const char *s)
{
    while (*s) lcd_data(lcd, (uint8_t)(*s++));
}
