#include "Relay.h"
/********************************************************************************************************
Function Name: Relay_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Relay_Init()
{

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);  //开启定时器15的PWM模式
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);  //开启定时器15的PWM模式
}


/********************************************************************************************************
Function Name: Relay_Set  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 设置继电器开度
********************************************************************************************************/

void Relay_Set(uint8_t id,float ratio)
{
  if(id==0x01)
	{
		TIM2->CCR3=(uint16_t)((float)TIM2->ARR*ratio);
	
	}else if(id==0x02)
	{
		TIM2->CCR4=(uint16_t)((float)TIM2->ARR*ratio);
	}

}





























