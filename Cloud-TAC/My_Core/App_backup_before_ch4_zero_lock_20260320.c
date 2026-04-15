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
         ????C??      ????BUG
*/
/*********************************************************************************************************/
#include "App.h"
/********************************************************************************************************
Function Name: System_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/

pid_t Motor1_Pid;  //???PID

pid_t YS_Motor1_Pid;  //???PID

extern DMA_HandleTypeDef hdma_tim3_ch1;
#define R04_MOTOR_COUNT 12U
#define R04_DEG2RAD(x) ((x) * 3.14159265f / 180.0f)
#define R04_RAD2DEG(x) ((x) * 180.0f / 3.14159265f)
#define R04_ARRAY_SIZE(arr) ((uint8_t)(sizeof(arr) / sizeof((arr)[0])))
#define R04_CMD_DEADBAND 0.20f
#define R04_REMOTE_ARM_MS 500U
#define R04_CONTROL_DISABLE_TH (-0.5f)
#define R04_CONTROL_ENABLE_TH  (0.5f)

static const uint8_t R04_MotorIds[R04_MOTOR_COUNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

typedef enum
{
	R04_ACTION_STAND = 0,
	R04_ACTION_FORWARD_CRAWL,
	R04_ACTION_BACKWARD_CRAWL,
	R04_ACTION_STRAFE_LEFT,
	R04_ACTION_STRAFE_RIGHT,
	R04_ACTION_TURN_LEFT,
	R04_ACTION_TURN_RIGHT
} R04_ActionId;

typedef struct
{
	// Position order matches R04_MotorIds: {1,2,3,4,5,6,7,8,9,10,11,12}
	float pos_deg[R04_MOTOR_COUNT];
	uint16_t hold_ms;
	float speed;
	float torque;
	float kp;
	float kd;
} R04_ActionStep;

typedef struct
{
	const R04_ActionStep *steps;
	uint8_t step_count;
	uint8_t loop;
} R04_ActionGroup;

typedef struct
{
	R04_ActionId action_id;
	uint8_t step_idx;
	uint16_t step_tick;
} R04_ActionRuntime;

#define R04_STEP(hold_ms, speed, torque, kp, kd, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12) \
	{{(m1), (m2), (m3), (m4), (m5), (m6), (m7), (m8), (m9), (m10), (m11), (m12)}, (hold_ms), (speed), (torque), (kp), (kd)}

static float R04_Abs(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static uint8_t R04_IsRemoteNeutral(void)
{
	return ((R04_Abs(Sbus_Value[2]) < R04_CMD_DEADBAND) &&
	        (R04_Abs(Sbus_Value[0]) < R04_CMD_DEADBAND) &&
	        (R04_Abs(Sbus_Value[3]) < R04_CMD_DEADBAND)) ? 1U : 0U;
}

static const R04_ActionStep R04_StandSteps[] =
{
	R04_STEP(300U, 0.35f, 0.0f, 8.0f, 0.3f,
	         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
};

static const R04_ActionStep R04_ForwardCrawlSteps[] =
{
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         0.0f, 12.0f, -18.0f, 0.0f, 0.0f, 12.0f,
	         -18.0f, 0.0f, 0.0f, 12.0f, -18.0f, 0.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         18.0f, 12.0f, -18.0f, 0.0f, 18.0f, 12.0f,
	         -18.0f, 0.0f, 18.0f, 12.0f, -18.0f, 0.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         18.0f, 0.0f, -18.0f, 0.0f, 18.0f, 0.0f,
	         -18.0f, 0.0f, 18.0f, 0.0f, -18.0f, 0.0f),
	R04_STEP(240U, 0.30f, 0.0f, 6.0f, 0.20f,
	         0.0f, 0.0f, -28.0f, 0.0f, 0.0f, 0.0f,
	         -28.0f, 0.0f, 0.0f, 0.0f, -28.0f, 0.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         -18.0f, 0.0f, 0.0f, 12.0f, -18.0f, 0.0f,
	         0.0f, 12.0f, -18.0f, 0.0f, 0.0f, 12.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         -18.0f, 0.0f, 18.0f, 12.0f, -18.0f, 0.0f,
	         18.0f, 12.0f, -18.0f, 0.0f, 18.0f, 12.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         -18.0f, 0.0f, 18.0f, 0.0f, -18.0f, 0.0f,
	         18.0f, 0.0f, -18.0f, 0.0f, 18.0f, 0.0f),
	R04_STEP(240U, 0.30f, 0.0f, 6.0f, 0.20f,
	         -28.0f, 0.0f, 0.0f, 0.0f, -28.0f, 0.0f,
	         0.0f, 0.0f, -28.0f, 0.0f, 0.0f, 0.0f),
};

static const R04_ActionStep R04_BackwardCrawlSteps[] =
{
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         0.0f, 12.0f, 18.0f, 0.0f, 0.0f, 12.0f,
	         18.0f, 0.0f, 0.0f, 12.0f, 18.0f, 0.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         -18.0f, 12.0f, 18.0f, 0.0f, -18.0f, 12.0f,
	         18.0f, 0.0f, -18.0f, 12.0f, 18.0f, 0.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         -18.0f, 0.0f, 18.0f, 0.0f, -18.0f, 0.0f,
	         18.0f, 0.0f, -18.0f, 0.0f, 18.0f, 0.0f),
	R04_STEP(240U, 0.30f, 0.0f, 6.0f, 0.20f,
	         0.0f, 0.0f, 28.0f, 0.0f, 0.0f, 0.0f,
	         28.0f, 0.0f, 0.0f, 0.0f, 28.0f, 0.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         18.0f, 0.0f, 0.0f, 12.0f, 18.0f, 0.0f,
	         0.0f, 12.0f, 18.0f, 0.0f, 0.0f, 12.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         18.0f, 0.0f, -18.0f, 12.0f, 18.0f, 0.0f,
	         -18.0f, 12.0f, 18.0f, 0.0f, -18.0f, 12.0f),
	R04_STEP(220U, 0.28f, 0.0f, 6.0f, 0.20f,
	         18.0f, 0.0f, -18.0f, 0.0f, 18.0f, 0.0f,
	         -18.0f, 0.0f, 18.0f, 0.0f, -18.0f, 0.0f),
	R04_STEP(240U, 0.30f, 0.0f, 6.0f, 0.20f,
	         28.0f, 0.0f, 0.0f, 0.0f, 28.0f, 0.0f,
	         0.0f, 0.0f, 28.0f, 0.0f, 0.0f, 0.0f),
};

static const R04_ActionStep R04_StrafeLeftSteps[] =
{
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         -18.0f, 14.0f, 12.0f, 0.0f, 18.0f, 14.0f,
	         -18.0f, 0.0f, 12.0f, 14.0f, 18.0f, 0.0f),
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         -28.0f, 0.0f, 18.0f, 0.0f, 28.0f, 0.0f,
	         -28.0f, 0.0f, 18.0f, 0.0f, 28.0f, 0.0f),
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         18.0f, 0.0f, -12.0f, 14.0f, -18.0f, 0.0f,
	         18.0f, 14.0f, -12.0f, 0.0f, -18.0f, 14.0f),
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         28.0f, 0.0f, -18.0f, 0.0f, -28.0f, 0.0f,
	         28.0f, 0.0f, -18.0f, 0.0f, -28.0f, 0.0f),
};

static const R04_ActionStep R04_StrafeRightSteps[] =
{
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         18.0f, 14.0f, -12.0f, 0.0f, -18.0f, 14.0f,
	         18.0f, 0.0f, -12.0f, 14.0f, -18.0f, 0.0f),
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         28.0f, 0.0f, -18.0f, 0.0f, -28.0f, 0.0f,
	         28.0f, 0.0f, -18.0f, 0.0f, -28.0f, 0.0f),
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         -18.0f, 0.0f, 12.0f, 14.0f, 18.0f, 0.0f,
	         -18.0f, 14.0f, 12.0f, 0.0f, 18.0f, 14.0f),
	R04_STEP(260U, 0.42f, 0.0f, 8.0f, 0.3f,
	         -28.0f, 0.0f, 18.0f, 0.0f, 28.0f, 0.0f,
	         -28.0f, 0.0f, 18.0f, 0.0f, 28.0f, 0.0f),
};

static const R04_ActionStep R04_TurnLeftSteps[] =
{
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         -14.0f, 14.0f, -14.0f, 0.0f, -14.0f, 14.0f,
	         14.0f, 0.0f, 14.0f, 14.0f, 14.0f, 0.0f),
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         -24.0f, 0.0f, -24.0f, 0.0f, -24.0f, 0.0f,
	         24.0f, 0.0f, 24.0f, 0.0f, 24.0f, 0.0f),
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         -14.0f, 0.0f, -14.0f, 14.0f, -14.0f, 0.0f,
	         14.0f, 14.0f, 14.0f, 0.0f, 14.0f, 14.0f),
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         -24.0f, 0.0f, -24.0f, 0.0f, -24.0f, 0.0f,
	         24.0f, 0.0f, 24.0f, 0.0f, 24.0f, 0.0f),
};

static const R04_ActionStep R04_TurnRightSteps[] =
{
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         14.0f, 14.0f, 14.0f, 0.0f, 14.0f, 14.0f,
	         -14.0f, 0.0f, -14.0f, 14.0f, -14.0f, 0.0f),
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         24.0f, 0.0f, 24.0f, 0.0f, 24.0f, 0.0f,
	         -24.0f, 0.0f, -24.0f, 0.0f, -24.0f, 0.0f),
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         14.0f, 0.0f, 14.0f, 14.0f, 14.0f, 0.0f,
	         -14.0f, 14.0f, -14.0f, 0.0f, -14.0f, 14.0f),
	R04_STEP(260U, 0.40f, 0.0f, 8.0f, 0.3f,
	         24.0f, 0.0f, 24.0f, 0.0f, 24.0f, 0.0f,
	         -24.0f, 0.0f, -24.0f, 0.0f, -24.0f, 0.0f),
};

static const R04_ActionGroup R04_ActionGroups[] =
{
	{R04_StandSteps,         R04_ARRAY_SIZE(R04_StandSteps),         1U},
	{R04_ForwardCrawlSteps,  R04_ARRAY_SIZE(R04_ForwardCrawlSteps),  1U},
	{R04_BackwardCrawlSteps, R04_ARRAY_SIZE(R04_BackwardCrawlSteps), 1U},
	{R04_StrafeLeftSteps,    R04_ARRAY_SIZE(R04_StrafeLeftSteps),    1U},
	{R04_StrafeRightSteps,   R04_ARRAY_SIZE(R04_StrafeRightSteps),   1U},
	{R04_TurnLeftSteps,      R04_ARRAY_SIZE(R04_TurnLeftSteps),      1U},
	{R04_TurnRightSteps,     R04_ARRAY_SIZE(R04_TurnRightSteps),     1U},
};

static R04_ActionRuntime R04_ActionState = {R04_ACTION_STAND, 0U, 0U};
static uint16_t R04_RemoteNeutralTick = 0U;
static uint8_t R04_RemoteArmed = 0U;
static uint8_t R04_ControlEnabled = 0U;
static uint8_t R04_ZeroPoseValid = 0U;
static float R04_ZeroPoseDeg[R04_MOTOR_COUNT] = {0.0f};
static R04_ActionStep R04_HoldCurrentStep =
{
	{0.0f},
	80U,
	0.12f,
	0.0f,
	3.0f,
	0.12f
};

static uint8_t R04_UpdateHoldPoseFromFeedback(void)
{
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
	{
		const Rs_Motor *motor = RobStride04_GetMotor(R04_MotorIds[idx]);
		if (motor == 0)
		{
			return 0U;
		}
		R04_HoldCurrentStep.pos_deg[idx] = R04_RAD2DEG(motor->position);
	}
	return 1U;
}

static uint8_t R04_UpdateZeroPoseFromFeedback(void)
{
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
	{
		const Rs_Motor *motor = RobStride04_GetMotor(R04_MotorIds[idx]);
		if (motor == 0)
		{
			return 0U;
		}
		R04_ZeroPoseDeg[idx] = R04_RAD2DEG(motor->position);
	}

	R04_ZeroPoseValid = 1U;
	return 1U;
}

static uint8_t R04_IsControlEnabled(void)
{
	if (Sbus_Value[4] > R04_CONTROL_ENABLE_TH)
	{
		R04_ControlEnabled = 1U;
	}
	else if (Sbus_Value[4] < R04_CONTROL_DISABLE_TH)
	{
		R04_ControlEnabled = 0U;
	}

	return R04_ControlEnabled;
}

static void R04_DisableAllMotors(void)
{
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
	{
		RobStride04_Set(R04_MotorIds[idx], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U);
	}
}

static R04_ActionId R04_SelectActionFromRemote(void)
{
	float forward_cmd = Sbus_Value[2];
	float strafe_cmd = Sbus_Value[0];
	float yaw_cmd = Sbus_Value[3];

	if (Sbus_Connect_Flag == 0U)
	{
		R04_RemoteNeutralTick = 0U;
		R04_RemoteArmed = 0U;
		R04_ControlEnabled = 0U;
		return R04_ACTION_STAND;
	}

	if (R04_IsControlEnabled() == 0U)
	{
		R04_RemoteNeutralTick = 0U;
		R04_RemoteArmed = 0U;
		return R04_ACTION_STAND;
	}

	if (R04_RemoteArmed == 0U)
	{
		if (R04_IsRemoteNeutral() == 0U)
		{
			R04_RemoteNeutralTick = 0U;
			return R04_ACTION_STAND;
		}

		if (R04_RemoteNeutralTick < R04_REMOTE_ARM_MS)
		{
			R04_RemoteNeutralTick++;
			return R04_ACTION_STAND;
		}

		R04_RemoteArmed = 1U;
	}

	if (yaw_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_TURN_RIGHT;
	}

	if (yaw_cmd < -R04_CMD_DEADBAND)
	{
		return R04_ACTION_TURN_LEFT;
	}

	if (strafe_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_STRAFE_RIGHT;
	}

	if (strafe_cmd < -R04_CMD_DEADBAND)
	{
		return R04_ACTION_STRAFE_LEFT;
	}

	if (forward_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_FORWARD_CRAWL;
	}

	if (forward_cmd < -R04_CMD_DEADBAND)
	{
		return R04_ACTION_BACKWARD_CRAWL;
	}

	if ((R04_Abs(forward_cmd) < R04_CMD_DEADBAND) &&
	    (R04_Abs(strafe_cmd) < R04_CMD_DEADBAND) &&
	    (R04_Abs(yaw_cmd) < R04_CMD_DEADBAND))
	{
		return R04_ACTION_STAND;
	}

	return R04_ACTION_STAND;
}

static void R04_ResetActionState(R04_ActionId action_id)
{
	R04_ActionState.action_id = action_id;
	R04_ActionState.step_idx = 0U;
	R04_ActionState.step_tick = 0U;
}

static void R04_ApplyActionStep(const R04_ActionStep *step)
{
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
	{
		float target_deg = step->pos_deg[idx];
		if (R04_ZeroPoseValid != 0U)
		{
			target_deg += R04_ZeroPoseDeg[idx];
		}

		RobStride04_Set(R04_MotorIds[idx],
		                R04_DEG2RAD(target_deg),
		                step->speed,
		                step->torque,
		                step->kp,
		                step->kd,
		                1);
	}
}
		
void System_Init()
{
	
//	HAL_Delay(5000);//???5S????????
//	
	
  Remote_Init(); //??????????
	Step_Motor_Init();//????????????
	Relay_Init();//??????????
	Pwm_Init(); //PWM?????
//	NSM_77_Init();//????????????
	RS485_Init();//RS485?????
	BRT38_Init();//??????????????
	
	YS_Motor_Init();//????????????
	
  WS2812B_Init(&htim3, &hdma_tim3_ch1);  //bling 
	
	
	Step_Pwm_Fre(5000);  //??????????????????
	Servo_Init();//????????
	
	Sbus_Init();
	//Ibus_Init();
	
	R04_filter(&hfdcan1,1);
	RobStride04_Manager_Init(R04_MotorIds, (uint8_t)(sizeof(R04_MotorIds) / sizeof(R04_MotorIds[0])));

  RM3508_Init();  //3508????????
	

	HAL_GPIO_WritePin(IN_1_GPIO_Port,IN_1_Pin,GPIO_PIN_RESET);  //A???
	HAL_GPIO_WritePin(IN_2_GPIO_Port,IN_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_3_GPIO_Port,IN_3_Pin,GPIO_PIN_RESET);  //B???
	HAL_GPIO_WritePin(IN_4_GPIO_Port,IN_4_Pin,GPIO_PIN_RESET);
	
	

	PID_struct_init(&Motor1_Pid,POSITION_PID,5.0f,0.2f,5.5f,0.005f,0.5f,10.0f,0.02f); //PID?????
	
	PID_struct_init(&YS_Motor1_Pid,POSITION_PID,5.0f,0.2f,5.5f,0.005f,0.5f,10.0f,0.02f); //PID?????	

	HAL_TIM_Base_Start_IT(&htim6); //所有外设初始化完成后，再启动1ms任务定时器
	
	
	
}





/********************************************************************************************************
Function Name: Mymain  ??????main????
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
Function Name: Millisecond_Task  ????????
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

								
extern Rs_Motor Rs_Motor_1;   //?????????1



uint8_t Lock_Flag=0x01;
int i=0;

uint8_t Motor_id=0x01;
float P_des=0.0f;  //???? ??????????+5.7????
float V_des=0.0f;
float T_ff=0.0f;
float Kp=0.5f;   //????  ??????3.5  ???????????0.1
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
//			Zero_Offset=Motor_2.pfb;//??????????????
//			i=0;    //1S????????? ????????????
//		}
//	
//	}else
//	{
//
//		
//	 YS_Set(Motor_id,P_des,V_des,T_ff,Kp,Kd,1);
//	
//	}
//	
//}

void MS_Task()
{
	R04_ActionId selected_action;
	const R04_ActionGroup *group;
	const R04_ActionStep *step;

	if (RobStride04_IsReady() == 0U)
	{
		return;
	}

	if ((Sbus_Connect_Flag == 0U) || (R04_IsControlEnabled() == 0U))
	{
		R04_ResetActionState(R04_ACTION_STAND);
		R04_RemoteNeutralTick = 0U;
		R04_RemoteArmed = 0U;
		R04_UpdateZeroPoseFromFeedback();
		R04_DisableAllMotors();
		return;
	}

	if ((R04_ZeroPoseValid == 0U) && (R04_UpdateZeroPoseFromFeedback() == 0U))
	{
		R04_DisableAllMotors();
		return;
	}

	selected_action = R04_SelectActionFromRemote();
	if (selected_action != R04_ActionState.action_id)
	{
		R04_ResetActionState(selected_action);
	}

	if (selected_action == R04_ACTION_STAND)
	{
		if (R04_UpdateHoldPoseFromFeedback() == 0U)
		{
			return;
		}
		R04_ApplyActionStep(&R04_HoldCurrentStep);
		return;
	}

	group = &R04_ActionGroups[selected_action];
	if (group->step_count == 0U)
	{
		return;
	}

	step = &group->steps[R04_ActionState.step_idx];
	R04_ApplyActionStep(step);

	R04_ActionState.step_tick++;
	if (R04_ActionState.step_tick >= step->hold_ms)
	{
		R04_ActionState.step_tick = 0U;
		if ((R04_ActionState.step_idx + 1U) < group->step_count)
		{
			R04_ActionState.step_idx++;
		}
		else if (group->loop != 0U)
		{
			R04_ActionState.step_idx = 0U;
		}
	}
}











void Millisecond_Task()      //1ms???????
{


	
	MS_Task();
	RobStride04_Manager_Task1ms();
	
	



}

/********************************************************************************************************
Function Name: Millisecond_10_Task  10????????
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Millisecond_50_Task()      //10ms???????
{
	
//	Position_Read(0x01,0x0000,0x0002);//??????????

}

/********************************************************************************************************
Function Name: HAL_TIM_PeriodElapsedCallback  ????????????
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
		Millisecond_Task();  //1ms????
		pluse++;
		if(pluse>=50)
		{
			Millisecond_50_Task();//50ms????
			pluse=0;
		}
	}
}















