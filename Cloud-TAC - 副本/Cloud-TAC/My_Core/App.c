/*********************************************************************************************************/
/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
         Blessings, no bugs
*/
/*********************************************************************************************************/
#include "App.h"
#include "RobStride04.h"
/********************************************************************************************************
Function Name: System_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/

pid_t Motor1_Pid;  // motor PID

pid_t YS_Motor1_Pid;  // motor PID

extern DMA_HandleTypeDef hdma_tim3_ch1;

/* Robot joint control -------------------------------------------------------*/
#define ROBOT_JOINT_NUM 12u

/* Target joint angle (deg). Index 0 -> motor ID 1 */
float Robot_Joint_Target_Pos[ROBOT_JOINT_NUM] = {0.0f};
/* Shared joint speed limit (rad/s). Change to an array if needed. */
float Robot_Joint_Target_Speed = 0.5f;

/* Save current position as zero for each joint motor */
static void Robot_Joint_Init(void)
{
	/* Let bus & motors settle after power-up */
	HAL_Delay(1000);

	for (uint8_t id = 1; id <= ROBOT_JOINT_NUM; id++)
	{
		Motor_SetControl(id);   /* set current position as zero */
		HAL_Delay(200);         /* wait for motor to process */
	}
}

/* Periodic joint control task (position control) */
static void Robot_Joint_Task(void)
{
	for (uint8_t id = 1; id <= ROBOT_JOINT_NUM; id++)
	{
		float angle_deg = Robot_Joint_Target_Pos[id - 1];
		Motor_Control(id, angle_deg, Robot_Joint_Target_Speed);
	}
}
		
void System_Init()
{
	
//	HAL_Delay(5000); // wait 5s after power-up
//	
	
	HAL_TIM_Base_Start_IT(&htim6); // start TIM6, 1ms interrupt
   Remote_Init(); // remote init
	Step_Motor_Init(); // step motor init
	Relay_Init(); // relay init
	Pwm_Init(); // PWM init
//	NSM_77_Init(); // radar init (optional)
	RS485_Init(); // RS485 init
	BRT38_Init(); // sensor init
	
	YS_Motor_Init(); // motor init
	
    WS2812B_Init(&htim3, &hdma_tim3_ch1);  //bling 
	
	
	Step_Pwm_Fre(5000);  // set step motor pulse frequency
	Servo_Init(); // servo init
	
	Sbus_Init(); // SBus init
	//Ibus_Init();
	
	R04_filter(&hfdcan1,1);

    RM3508_Init();  // RM3508 motor init
	

	HAL_GPIO_WritePin(IN_1_GPIO_Port,IN_1_Pin,GPIO_PIN_RESET);  // A output
	HAL_GPIO_WritePin(IN_2_GPIO_Port,IN_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_3_GPIO_Port,IN_3_Pin,GPIO_PIN_RESET);  // B output
	HAL_GPIO_WritePin(IN_4_GPIO_Port,IN_4_Pin,GPIO_PIN_RESET);
	
	

	PID_struct_init(&Motor1_Pid,POSITION_PID,5.0f,0.2f,5.5f,0.005f,0.5f,10.0f,0.02f); // PID init
	
	PID_struct_init(&YS_Motor1_Pid,POSITION_PID,5.0f,0.2f,5.5f,0.005f,0.5f,10.0f,0.02f); // PID init	

	/* Joint zeroing (save current mechanical position as 0) */
	Robot_Joint_Init();
}





/********************************************************************************************************
Function Name: Mymain  custom main function
Author       : ZFY
Date         : 2023-06-09
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
									
			
											
void Mymain()
{
	
	System_Init();

			for(;;)
			{

				HAL_GPIO_TogglePin(Led_1_GPIO_Port,Led_1_Pin);
	
                WS2812B_RainbowEffect(&htim3, &hdma_tim3_ch1,1.0f);

			

      }

	
	
}




/********************************************************************************************************
Function Name: Millisecond_Task  1ms tasks
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/





float Speed_Set=0.0f;

uint8_t Key_Flag=0;	
uint8_t Key_Down_Flag=0;	

float torque_set=0.0f;

								
extern Rs_Motor Rs_Motor_1;   // joint motor 1



uint8_t Lock_Flag=0x01;
int i=0;

uint8_t Motor_id=0x01;
float P_des=0.0f;  // position (application specific)
float V_des=0.0f;
float T_ff=0.0f;
float Kp=0.5f;   // position kp
float Kd=0.02f;
float Zero_Offset=0.0f;

float Pos_Set=0.0f;

float Pos_InV=-0.0f;

//void MS_Task()
//{
//	
//	if(Sbus_Value[7]<=0.0f) 
//	{
//		i++;
//		if(i>1000)
//		{
//		  YS_Set(Motor_id,0.0f,0.0f,0.0f,0.0f,0.0f,0);
//			Zero_Offset=Motor_2.pfb; // position offset
//			i=0;    // lock once per second to avoid issues
//		}
//	
//	}else
//	{

//		
//	 YS_Set(Motor_id,P_des,V_des,T_ff,Kp,Kd,1);
//	
//	}
//	
//}

void MS_Task()
{
//   T_ff=pid_calc(&YS_Motor1_Pid,Motor_2.pfb,Sbus_Value[2]);
   //T_ff=Sbus_Value[2];
		//P_des=Sbus_Value[2]*6.33f*PI/2.0f+Zero_Offset;
//	P_des=Pos_Set*6.33f*PI/2.0f+Zero_Offset;
	YS_Set(Motor_id,P_des,V_des,T_ff,Kp,Kd,1);
}











void Millisecond_Task()      // 1ms loop
{


	
	MS_Task();
	
	



}

/********************************************************************************************************
Function Name: Millisecond_10_Task  10ms tasks (actually 50ms here)
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Millisecond_50_Task()      // 50ms loop
{
	
	/* Joint control loop (about 50ms) */
	Robot_Joint_Task();

}

/********************************************************************************************************
Function Name: HAL_TIM_PeriodElapsedCallback  TIM callback
Author       : ZFY
Date         : 2024-01-05
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/

int pluse=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM6)
	{
		Millisecond_Task();  // 1ms tasks
		pluse++;
		if(pluse>=50)
		{
			Millisecond_50_Task(); // 50ms tasks
			pluse=0;
		}
	}
}















