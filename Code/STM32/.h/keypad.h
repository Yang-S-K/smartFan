#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32f4xx_hal.h"

// 定義 4x4 矩陣鍵盤按鍵對應表
extern const char keyMap[4][4];

// 初始化鍵盤 GPIO
void Keypad_Init(void);

// 掃描矩陣鍵盤並回傳按鍵值
char Keypad_Scan(void);

#endif // KEYPAD_H
