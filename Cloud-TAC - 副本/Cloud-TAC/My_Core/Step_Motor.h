#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H





#include "main.h"




void Step_Motor_Init(void); //步进电机初始化

void Step_Pwm_Fre(uint32_t _fre); //步进电机PWM波频率（步进电机速度）设置

void Step_Motor_CW(uint8_t id); 
void Step_Motor_CCW(uint8_t id);
void Step_Motor_Stop(uint8_t id);




#endif //STEP_MOTOR_H
