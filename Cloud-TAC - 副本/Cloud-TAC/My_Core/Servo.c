
#include "Servo.h"
/********************************************************************************************************
Function Name: Servo_Init  
Author       : ZFY
Date         : 2025-04-23
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Servo_Init()
{
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);  //开启定时器4的PWM模式
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);  //开启定时器4的PWM模式
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  //开启定时器4的PWM模式
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);  //开启定时器4的PWM模式   用于舵机控制
	
	
  TIM4->CCR1=1500;	
	TIM4->CCR2=1500;	
	TIM4->CCR3=1500;	
	TIM4->CCR4=1500;	

}


void Servo_Set_Deg(uint8_t _id,float _deg)
{
	
	switch(_id)
		{
       case 0x01:
       TIM4->CCR1=500+_deg*2000/270;
       break; 
       case 0x02:
       TIM4->CCR2=500+_deg*2000/180;
       break; 
       case 0x03:
       TIM4->CCR3=500+_deg*2000/180;
       break; 
			 case 0x04:
       TIM4->CCR4=500+_deg*2000/180;
       break; 

     }
}































