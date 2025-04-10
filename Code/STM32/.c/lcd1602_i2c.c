#include "lcd1602_i2c.h"
#include "main.h"
#include <string.h>

// LCD I2C 地址 (0x27)
#define LCD_I2C_ADDR  (0x27 << 1)  // STM32 HAL 需要 7-bit 地址 (左移 1 bit)

// LCD 指令
#define LCD_CMD  0
#define LCD_DATA 1
#define LCD_BACKLIGHT 0x08  // 背光開啟
#define LCD_ENABLE 0x04  // 使能位

extern I2C_HandleTypeDef hi2c1; // I2C 句柄 (視 CubeMX 設定可能需要更改)

// 延遲函式
static void LCD_Delay(uint16_t ms) {
    HAL_Delay(ms);
}

// 透過 I2C 傳送資料
static void LCD_Send(uint8_t data, uint8_t mode) {
    uint8_t high_nibble = (data & 0xF0) | LCD_BACKLIGHT | mode;
    uint8_t low_nibble = ((data << 4) & 0xF0) | LCD_BACKLIGHT | mode;
    uint8_t data_buffer[4] = {
        high_nibble | LCD_ENABLE,
        high_nibble,
        low_nibble | LCD_ENABLE,
        low_nibble
    };
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, data_buffer, 4, HAL_MAX_DELAY);
}

// 傳送 LCD 指令
void LCD_SendCommand(uint8_t cmd) {
    LCD_Send(cmd, LCD_CMD);
    LCD_Delay(2);
}

// 傳送 LCD 資料
void LCD_SendData(uint8_t data) {
    LCD_Send(data, LCD_DATA);
    LCD_Delay(2);
}

// LCD 初始化
void LCD_Init() {
    LCD_Delay(50);
    LCD_SendCommand(0x30);
    LCD_Delay(5);
    LCD_SendCommand(0x30);
    LCD_Delay(1);
    LCD_SendCommand(0x30);
    LCD_Delay(10);

    LCD_SendCommand(0x20); // 設定 4-bit 模式
    LCD_SendCommand(0x28); // 4-bit, 2 行, 5x8 字型
    LCD_SendCommand(0x0C); // 顯示開啟, 不顯示游標, 不閃爍
    LCD_SendCommand(0x06); // 文字靠左, 游標自動遞增
    LCD_SendCommand(0x01); // 清除顯示
    LCD_Delay(2);
}

// 清除 LCD 顯示
void LCD_Clear() {
    LCD_SendCommand(0x01);
    LCD_Delay(2);
}

// 設定游標位置 (row: 0-1, col: 0-15)
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40};
    LCD_SendCommand(0x80 | (col + row_offsets[row]));
}

// 顯示字串
void LCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}
