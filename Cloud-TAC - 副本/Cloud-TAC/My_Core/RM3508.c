
#include "RM3508.h"



FDCAN_FilterTypeDef RM3508_sFilterConfig;
FDCAN_TxHeaderTypeDef RM3508_TxHeader;

static Motor_Feedback_t motor_feedback[MAX_MOTORS];
static int16_t target_currents[MAX_MOTORS] = {0};
float Motor_Speed[MAX_MOTORS];
float Motor_Speed_Last[MAX_MOTORS];

pid_t RM3508_1_Speed_Pid;  //电机PID
pid_t RM3508_2_Speed_Pid;  //电机PID
pid_t RM3508_3_Speed_Pid;  //电机PID
pid_t RM3508_4_Speed_Pid;  //电机PID   电机速度PID



/********************************************************************************************************
Function Name: RM3508_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void RM3508_Init()
{
	RM3508_filter(&hfdcan2,1,0x01); //3508电机初始化
	RM3508_filter(&hfdcan2,2,0x02); //3508电机初始化
	RM3508_filter(&hfdcan2,3,0x03); //3508电机初始化
	RM3508_filter(&hfdcan2,4,0x04); //3508电机初始化 //电机过滤器初始化
	
	PID_struct_init(&RM3508_1_Speed_Pid,POSITION_PID,100.0f,10.0f,5.5f,0.005f,0.5f,400.0f,0.2f); //PID初始化
  PID_struct_init(&RM3508_2_Speed_Pid,POSITION_PID,100.0f,10.0f,5.5f,0.005f,0.5f,400.0f,0.2f); //PID初始化
	PID_struct_init(&RM3508_3_Speed_Pid,POSITION_PID,100.0f,10.0f,5.5f,0.005f,0.5f,400.0f,0.2f); //PID初始化
	PID_struct_init(&RM3508_4_Speed_Pid,POSITION_PID,100.0f,10.0f,5.5f,0.005f,0.5f,400.0f,0.2f); //PID初始化
}






void RM3508_filter(FDCAN_HandleTypeDef* fdcanhandle,uint32_t FilterBank_Num,uint32_t _Id)
{
	
	RM3508_sFilterConfig.IdType=FDCAN_STANDARD_ID;//标准ID
	RM3508_sFilterConfig.FilterIndex=FilterBank_Num;//滤波器索引   
	RM3508_sFilterConfig.FilterType=FDCAN_FILTER_MASK;      //滤波器类型
	RM3508_sFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO1;//过滤器0关联到FIFO1       CAN2关联到了FIFO1
	RM3508_sFilterConfig.FilterID1 =CAN_ID_TX+_Id; 
	RM3508_sFilterConfig.FilterID2 =0x7FF;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &RM3508_sFilterConfig);

	RM3508_TxHeader.IdType=FDCAN_STANDARD_ID;                  //扩展ID
	RM3508_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
	RM3508_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	RM3508_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
	RM3508_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
	RM3508_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
	RM3508_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
	RM3508_TxHeader.MessageMarker=0;  
	HAL_FDCAN_ConfigFilter(fdcanhandle,&RM3508_sFilterConfig);//滤波器初始化
	HAL_FDCAN_ConfigGlobalFilter(fdcanhandle,FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);//FDCAN_ACCEPT_IN_RX_FIFO0
	HAL_FDCAN_ActivateNotification(fdcanhandle,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
	HAL_FDCAN_Start(&hfdcan2);  /* Start the FDCAN module */	
}



uint8_t FDCAN2_Send_Msg(uint8_t* msg,uint32_t CAN_ID)
{	
	RM3508_TxHeader.Identifier=CAN_ID;                           //ID
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&RM3508_TxHeader,msg)!=HAL_OK) {
		return 1;}//发送}
    return 0;	
}






// 设置目标电流（-16384 ~ 16384对应-20A~20A）
void RM3508_Set_Current(uint8_t motor_id, float _current)   //输入电流值范围为-100 - 100
{
	
	int16_t current=(int16_t)(_current*163.84f);
	
    if(motor_id >= 1 && motor_id <= MAX_MOTORS) {
        target_currents[motor_id-1] = __SSAT(current, 16);
    }
}

// 发送控制命令
void RM3508_Send_Current(void)
{
    uint8_t data[8];
    // 构造数据包（每个电机2字节，小端模式）
    for(int i=0; i<4; i++) {
        data[i*2] = target_currents[i] >> 8;
        data[i*2+1] = target_currents[i] & 0xFF;
    }
    
	RM3508_TxHeader.Identifier=CAN_ID_TX;             //ID
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&RM3508_TxHeader,data);	

}

// 接收反馈处理（在CAN中断回调中调用）
void RM3508_DataTransform(FDCAN_RxHeaderTypeDef rx_header,uint8_t* _rxdate)
{
        uint8_t motor_id = rx_header.FilterIndex - CAN_ID_RX_BASE;
        if(motor_id < MAX_MOTORS) {
					
            motor_feedback[motor_id].id = motor_id + 1;
            motor_feedback[motor_id].angle = (_rxdate[0] << 8) | _rxdate[1];
            motor_feedback[motor_id].speed = (_rxdate[2] << 8) | _rxdate[3];
            motor_feedback[motor_id].torque_current = (_rxdate[4] << 8) | _rxdate[5];	
					  Motor_Speed[motor_id]=0.5f*(motor_feedback[motor_id].speed/100.0f)+0.5f*Motor_Speed_Last[motor_id];	//数据滤波
            Motor_Speed_Last[motor_id]=	Motor_Speed[motor_id];				
        }			

}




void Mecanum_wheel_chassis_control()
{
		mecanum_control(Sbus_Value[1]*100.0f,-Sbus_Value[0]*100.0f,Sbus_Value[3]*100.0f);  //麦克纳姆轮速度解算
		RM3508_Set_Current(1,pid_calc(&RM3508_1_Speed_Pid,Motor_Speed[0],wheel_speeds.front_left));
		RM3508_Set_Current(2,pid_calc(&RM3508_2_Speed_Pid,Motor_Speed[1],wheel_speeds.front_right));
		RM3508_Set_Current(3,pid_calc(&RM3508_3_Speed_Pid,Motor_Speed[2],wheel_speeds.rear_left));
		RM3508_Set_Current(4,pid_calc(&RM3508_4_Speed_Pid,Motor_Speed[3],wheel_speeds.rear_right));
		RM3508_Send_Current();//向电机发送电流值

}










