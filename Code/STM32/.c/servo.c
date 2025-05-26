#include "servo.h"

extern TIM_HandleTypeDef htim4;


char strServo[100];


void Calibrate_Servo_Zero(void) {
	    Set_Servo_Angle(90, 180); // 讓 180° 伺服馬達回正中間
	    Set_Servo_Angle(180, 360); // 讓 360° 伺服馬達回正中間（停止狀態）
}


void Set_Servo_Angle(int angle,int whichServo) {
    uint16_t min_pulse = 500;   // 調整最小 PWM 脈衝（原本是 500）
    uint16_t max_pulse = 2600;  // 調整最大 PWM 脈衝（原本是 2600）
    uint16_t pulse = min_pulse + (angle * (max_pulse - min_pulse) / 180);
    if(whichServo==180){
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);
    }else{//最多轉180
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse);
    }

}

