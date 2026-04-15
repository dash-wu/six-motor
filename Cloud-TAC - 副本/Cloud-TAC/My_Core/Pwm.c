
#include "Pwm.h"
/********************************************************************************************************
Function Name: Pwm_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Pwm_Init()
{
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);  //开启定时器4的PWM模式
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);  //开启定时器4的PWM模式
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  //开启定时器4的PWM模式
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);  //开启定时器4的PWM模式   用于舵机控制

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);  //开启定时器2的PWM模式
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);  //开启定时器2的PWM模式
}






























