
#include "Step_Motor.h"
/********************************************************************************************************
Function Name: Step_Motor_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Step_Motor_Init()
{
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);  //开启定时器15的PWM模式
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);  //开启定时器15的PWM模式
	
}


/********************************************************************************************************
Function Name: Step_Pwm_Fre  
Author       : ZFY
Date         : 2025-01-20
Description  :
Outputs      : void
Notes        : 步进电机PWN频率设定  最低305HZ
********************************************************************************************************/
void Step_Pwm_Fre(uint32_t _fre)
{
	
	uint16_t arr=(uint16_t)(1000000/_fre);
	if((arr>1)&&(arr<65535))
	{
		TIM15->ARR=arr;
	}	
}

/********************************************************************************************************
Function Name: Step_Motor_CW  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 控制电机正转
********************************************************************************************************/
void Step_Motor_CW(uint8_t id)
{
	if(id==0x01)
	{
		TIM15->CCR1=TIM15->ARR/2;
	  HAL_GPIO_WritePin(Step_D1_GPIO_Port,Step_D1_Pin,GPIO_PIN_RESET);
	}else if(id==0x02)
	{
		TIM15->CCR2=TIM15->ARR/2;
	  HAL_GPIO_WritePin(Step_D2_GPIO_Port,Step_D2_Pin,GPIO_PIN_RESET);			
	}
}


/********************************************************************************************************
Function Name: Step_Motor_CW  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 控制电机反转
********************************************************************************************************/
void Step_Motor_CCW(uint8_t id)
{
	if(id==0x01)
	{
		TIM15->CCR1=TIM15->ARR/2;
	  HAL_GPIO_WritePin(Step_D1_GPIO_Port,Step_D1_Pin,GPIO_PIN_SET);
	}else if(id==0x02)
	{
		TIM15->CCR2=TIM15->ARR/2;
	  HAL_GPIO_WritePin(Step_D2_GPIO_Port,Step_D2_Pin,GPIO_PIN_SET);			
	}
}

/********************************************************************************************************
Function Name: Step_Motor_Step  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 控制电机停转
********************************************************************************************************/
void Step_Motor_Stop(uint8_t id)
{
	if(id==0x01)
	{
		TIM15->CCR1=0;
	  HAL_GPIO_WritePin(Step_D1_GPIO_Port,Step_D1_Pin,GPIO_PIN_RESET);
	}else if(id==0x02)
	{
		TIM15->CCR2=0;
	  HAL_GPIO_WritePin(Step_D2_GPIO_Port,Step_D2_Pin,GPIO_PIN_RESET);			
	}
}























