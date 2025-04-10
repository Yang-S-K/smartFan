#ifndef ESP32_H
#define ESP32_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;

typedef struct {
    int fanState;
    int fanSpeed;
    int hAngle;
    int vAngle;
} FanData;

// **函式宣告**
void ESP32_UART_Receive(void);
void SendDataToESP32(int fanState, int fanSpeed, int hAngle, int vAngle,float temperature);
FanData ESP32_GetFanData(void);

#endif /* ESP32_H */
