#include "motor.h"

static TIM_HandleTypeDef *motor_htim;  // 定義 PWM 定時器變數

// 初始化馬達（PWM）
void Motor_Init(TIM_HandleTypeDef *htim) {
    motor_htim = htim;  // 記住 PWM 定時器
    HAL_TIM_PWM_Start(motor_htim, MOTOR_PWM_CHANNEL);  // 啟動 PWM
}

// 設定馬達轉速（0~100%）
void Motor_SetSpeed(uint8_t speed) {
    if (speed > 100) speed = 100;  // 限制速度範圍
    __HAL_TIM_SET_COMPARE(motor_htim, MOTOR_PWM_CHANNEL, speed);
}

// 停止馬達
void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(motor_htim, MOTOR_PWM_CHANNEL, 0);  // 設定 PWM 0%
}
