#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"

// 定義馬達 PWM 控制腳位
#define MOTOR_PWM_CHANNEL TIM_CHANNEL_1  // PA8 (TIM1_CH1)

// 初始化馬達（PWM）
void Motor_Init(TIM_HandleTypeDef *htim);

// 設定馬達轉速（0% ~ 100%）
void Motor_SetSpeed(uint8_t speed);

// 停止馬達
void Motor_Stop(void);

#endif // MOTOR_H