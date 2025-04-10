#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

void Set_Servo_Angle(int angle,int whichServo);
void Calibrate_Servo_Zero(void);

#endif // SERVO_H
