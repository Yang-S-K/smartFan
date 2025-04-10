#ifndef LCD1602_I2C_H_
#define LCD1602_I2C_H_

#include "stm32f4xx_hal.h"

// 函式宣告
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);

#endif /* LCD1602_I2C_H_ */
