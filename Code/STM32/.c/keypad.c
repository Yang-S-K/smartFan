#include "keypad.h"

// 定義鍵盤對應的按鍵
const char keyMap[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// GPIO 腳位對應 (根據 STM32CubeMX 設定)
GPIO_TypeDef* ROW_PORTS[4] = {GPIOC, GPIOC, GPIOC, GPIOC}; // PC8-PC11
uint16_t ROW_PINS[4] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11};

GPIO_TypeDef* COL_PORTS[4] = {GPIOC, GPIOD, GPIOG, GPIOG}; // PC12, PD2, PG2, PG3
uint16_t COL_PINS[4] = {GPIO_PIN_12, GPIO_PIN_2, GPIO_PIN_2, GPIO_PIN_3};
// 初始化鍵盤 GPIO
void Keypad_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 啟用 GPIO 時鐘
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    // 設定 ROWS 為輸出模式
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    for (int i = 0; i < 4; i++) {
        GPIO_InitStruct.Pin = ROW_PINS[i];
        HAL_GPIO_Init(ROW_PORTS[i], &GPIO_InitStruct);
        HAL_GPIO_WritePin(ROW_PORTS[i], ROW_PINS[i], GPIO_PIN_SET); // 預設為高
    }

    // 設定 COLS 為輸入模式，啟用 Pull-up
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    for (int i = 0; i < 4; i++) {
        GPIO_InitStruct.Pin = COL_PINS[i];
        HAL_GPIO_Init(COL_PORTS[i], &GPIO_InitStruct);
    }
}

// 掃描鍵盤
char Keypad_Scan(void) {
    static char lastKey = 0;  // 記錄上次按下的鍵
    char key = 0;             // 當前偵測到的鍵

    for (int row = 0; row < 4; row++) {
        // 1. 設定當前列為低電位，其他列恢復高電位
        for (int i = 0; i < 4; i++) {
            HAL_GPIO_WritePin(ROW_PORTS[i], ROW_PINS[i], GPIO_PIN_SET);
        }
        HAL_GPIO_WritePin(ROW_PORTS[row], ROW_PINS[row], GPIO_PIN_RESET);

        // 2. 掃描每一個行的狀態
        for (int col = 0; col < 4; col++) {
            if (HAL_GPIO_ReadPin(COL_PORTS[col], COL_PINS[col]) == GPIO_PIN_RESET) {
                HAL_Delay(50);  // 去彈跳
                if (HAL_GPIO_ReadPin(COL_PORTS[col], COL_PINS[col]) == GPIO_PIN_RESET) {
                    key = keyMap[row][col];  // 紀錄按鍵
                }
            }
        }
    }

    // 3. 確保只回報新按鍵，避免重複輸出
    if (key != 0 && key != lastKey) {
        lastKey = key;  // 更新上次按鍵
        return key;
    } else if (key == 0) {
        lastKey = 0;  // 釋放按鍵後重置
    }

    return 0;  // 無新按鍵
}

