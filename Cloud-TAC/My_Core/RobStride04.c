
#include "RobStride04.h"


FDCAN_FilterTypeDef R04_sFilterConfig;
FDCAN_TxHeaderTypeDef R04_TxHeader;
FDCAN_RxHeaderTypeDef R04_RxHeader;
Rs_Motor Rs_Motor_List[RS_MOTOR_ID_MAX];
uint8_t Rs_Motor_Feedback_Valid[RS_MOTOR_ID_MAX] = {0};

typedef enum
{
	R04_MANAGER_UNINIT = 0,
	R04_MANAGER_BOOT_WAIT,
	R04_MANAGER_ZERO_SEND,
	R04_MANAGER_ZERO_WAIT,
	R04_MANAGER_ENABLE_SEND,
	R04_MANAGER_ENABLE_WAIT,
	R04_MANAGER_READY
} R04_ManagerState;

typedef struct
{
	uint8_t configured;
	uint8_t enabled;
	uint8_t control_active;
	uint8_t pending_enable;
	uint8_t pending_disable;
	float p_des;
	float v_des;
	float t_ff;
	float kp;
	float kd;
} R04_CommandSlot;

static R04_ManagerState R04_Manager_Status = R04_MANAGER_UNINIT;
static R04_CommandSlot R04_Command_List[RS_MOTOR_ID_MAX];
static uint8_t R04_Managed_MotorIds[RS_MOTOR_ID_MAX] = {0};
static uint8_t R04_Managed_MotorCount = 0;
static uint8_t R04_Startup_Index = 0;
static uint8_t R04_Send_Index = 0;
static uint32_t R04_State_Timestamp = 0;

static uint32_t R04_DlcFromLength(uint32_t len)
{
	switch (len)
	{
		case 0U: return FDCAN_DLC_BYTES_0;
		case 1U: return FDCAN_DLC_BYTES_1;
		case 2U: return FDCAN_DLC_BYTES_2;
		case 3U: return FDCAN_DLC_BYTES_3;
		case 4U: return FDCAN_DLC_BYTES_4;
		case 5U: return FDCAN_DLC_BYTES_5;
		case 6U: return FDCAN_DLC_BYTES_6;
		case 7U: return FDCAN_DLC_BYTES_7;
		default: return FDCAN_DLC_BYTES_8;
	}
}

static uint8_t R04_IsManagedMotor(uint8_t motor_id)
{
	if (motor_id >= RS_MOTOR_ID_MAX)
	{
		return 0U;
	}
	return R04_Command_List[motor_id].configured;
}

void R04_filter(FDCAN_HandleTypeDef* fdcanhandle,uint32_t FilterBank_Num)
{
	
	R04_sFilterConfig.IdType=FDCAN_EXTENDED_ID;//��չID
	R04_sFilterConfig.FilterIndex=FilterBank_Num;//�˲�������   
	R04_sFilterConfig.FilterType=FDCAN_FILTER_MASK;      //�˲�������
	R04_sFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;//������0������FIFO0       CAN1��������FIFO0 
	R04_sFilterConfig.FilterID1 =0x00000000; 
	R04_sFilterConfig.FilterID2 =0x00000000;
	
	R04_TxHeader.IdType=FDCAN_EXTENDED_ID;                  //��չID
	R04_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
	R04_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
	R04_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
	R04_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
	R04_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
	R04_TxHeader.MessageMarker=0;  

	 
	
	if (HAL_FDCAN_ConfigFilter(fdcanhandle, &R04_sFilterConfig) != HAL_OK)
	{
		return;
	}
	if (HAL_FDCAN_ConfigGlobalFilter(fdcanhandle, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK)
	{
		return;
	}
	if (HAL_FDCAN_ActivateNotification(fdcanhandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		return;
	}
	HAL_FDCAN_Start(fdcanhandle);                               //����FDCAN

}



uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len,uint32_t CAN_ID)
{	
	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
	{
		return 1;
	}
	R04_TxHeader.Identifier=CAN_ID;                           //32λID
	R04_TxHeader.DataLength=R04_DlcFromLength(len);           //���ݳ���
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&R04_TxHeader,msg)!=HAL_OK) {
		return 1;}//����}
    return 0;	
}

void RobStride04_Manager_Init(const uint8_t *motor_ids, uint8_t motor_count)
{
	R04_Managed_MotorCount = 0U;
	R04_Startup_Index = 0U;
	R04_Send_Index = 0U;
	R04_State_Timestamp = HAL_GetTick();

	for (uint8_t idx = 0; idx < RS_MOTOR_ID_MAX; idx++)
	{
		R04_Managed_MotorIds[idx] = 0U;
		R04_Command_List[idx].configured = 0U;
		R04_Command_List[idx].enabled = 0U;
		R04_Command_List[idx].control_active = 0U;
		R04_Command_List[idx].pending_enable = 0U;
		R04_Command_List[idx].pending_disable = 0U;
		R04_Command_List[idx].p_des = 0.0f;
		R04_Command_List[idx].v_des = 0.0f;
		R04_Command_List[idx].t_ff = 0.0f;
		R04_Command_List[idx].kp = 0.0f;
		R04_Command_List[idx].kd = 0.0f;
	}

	if (motor_ids == 0)
	{
		R04_Manager_Status = R04_MANAGER_UNINIT;
		return;
	}

	for (uint8_t idx = 0; (idx < motor_count) && (idx < RS_MOTOR_ID_MAX); idx++)
	{
		uint8_t motor_id = motor_ids[idx];
		if ((motor_id == 0U) || (motor_id >= RS_MOTOR_ID_MAX))
		{
			continue;
		}
		R04_Managed_MotorIds[R04_Managed_MotorCount++] = motor_id;
		R04_Command_List[motor_id].configured = 1U;
	}

	if (R04_Managed_MotorCount == 0U)
	{
		R04_Manager_Status = R04_MANAGER_UNINIT;
		return;
	}

	R04_Manager_Status = R04_MANAGER_BOOT_WAIT;
}

uint8_t RobStride04_IsReady(void)
{
	return (R04_Manager_Status == R04_MANAGER_READY) ? 1U : 0U;
}

HAL_StatusTypeDef RobStride04_Set(uint8_t motor_id, float p_des, float v_des, float t_ff, float kp, float kd, uint8_t status)
{
	if (R04_IsManagedMotor(motor_id) == 0U)
	{
		return HAL_ERROR;
	}

	if (status > 1U)
	{
		return HAL_ERROR;
	}

	if (status == 0U)
	{
		R04_Command_List[motor_id].control_active = 0U;
		if (R04_Command_List[motor_id].enabled != 0U)
		{
			R04_Command_List[motor_id].pending_disable = 1U;
			R04_Command_List[motor_id].pending_enable = 0U;
			R04_Command_List[motor_id].enabled = 0U;
		}
		return HAL_OK;
	}

	R04_Command_List[motor_id].p_des = p_des;
	R04_Command_List[motor_id].v_des = v_des;
	R04_Command_List[motor_id].t_ff = t_ff;
	R04_Command_List[motor_id].kp = kp;
	R04_Command_List[motor_id].kd = kd;
	R04_Command_List[motor_id].control_active = 1U;

	if ((RobStride04_IsReady() != 0U) && (R04_Command_List[motor_id].enabled == 0U))
	{
		R04_Command_List[motor_id].pending_enable = 1U;
		R04_Command_List[motor_id].pending_disable = 0U;
		R04_Command_List[motor_id].enabled = 1U;
	}

	return HAL_OK;
}

void RobStride04_Manager_Task1ms(void)
{
	const uint32_t boot_wait_ms = 3000U;
	const uint32_t zero_wait_ms = 1500U;
	const uint32_t settle_wait_ms = 1000U;
	uint32_t now = HAL_GetTick();

	if ((R04_Manager_Status == R04_MANAGER_UNINIT) || (R04_Managed_MotorCount == 0U))
	{
		return;
	}

	switch (R04_Manager_Status)
	{
		case R04_MANAGER_BOOT_WAIT:
		{
			if (now >= boot_wait_ms)
			{
				R04_Manager_Status = R04_MANAGER_ENABLE_SEND;
				R04_Startup_Index = 0U;
			}
		} break;

		case R04_MANAGER_ZERO_SEND:
		{
			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
			{
				return;
			}
			Motor_Cmd(R04_Managed_MotorIds[R04_Startup_Index], RS_ZERO);
			R04_Startup_Index++;
			if (R04_Startup_Index >= R04_Managed_MotorCount)
			{
				R04_Startup_Index = 0U;
				R04_State_Timestamp = now;
				R04_Manager_Status = R04_MANAGER_ZERO_WAIT;
			}
		} break;

		case R04_MANAGER_ZERO_WAIT:
		{
			if ((now - R04_State_Timestamp) >= zero_wait_ms)
			{
				R04_Manager_Status = R04_MANAGER_ENABLE_SEND;
				R04_Startup_Index = 0U;
			}
		} break;

		case R04_MANAGER_ENABLE_SEND:
		{
			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
			{
				return;
			}
			Motor_Cmd(R04_Managed_MotorIds[R04_Startup_Index], RS_ENABLE);
			R04_Command_List[R04_Managed_MotorIds[R04_Startup_Index]].enabled = 1U;
			R04_Startup_Index++;
			if (R04_Startup_Index >= R04_Managed_MotorCount)
			{
				R04_Startup_Index = 0U;
				R04_State_Timestamp = now;
				R04_Send_Index = 0U;
				R04_Manager_Status = R04_MANAGER_ENABLE_WAIT;
			}
		} break;

		case R04_MANAGER_ENABLE_WAIT:
		{
			if ((now - R04_State_Timestamp) >= settle_wait_ms)
			{
				R04_Manager_Status = R04_MANAGER_READY;
			}
		} break;

		case R04_MANAGER_READY:
		{
			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
			{
				return;
			}

			for (uint8_t attempt = 0; attempt < R04_Managed_MotorCount; attempt++)
			{
				uint8_t motor_id = R04_Managed_MotorIds[R04_Send_Index];
				R04_CommandSlot *slot = &R04_Command_List[motor_id];

				R04_Send_Index++;
				if (R04_Send_Index >= R04_Managed_MotorCount)
				{
					R04_Send_Index = 0U;
				}

				if (slot->pending_disable != 0U)
				{
					Motor_Cmd(motor_id, RS_DISABLE);
					slot->pending_disable = 0U;
					break;
				}

				if (slot->pending_enable != 0U)
				{
					Motor_Cmd(motor_id, RS_ENABLE);
					slot->pending_enable = 0U;
					break;
				}

				if (slot->control_active != 0U)
				{
					Motor_Ctrl(motor_id, slot->t_ff, slot->p_des, slot->v_des, slot->kp, slot->kd);
					break;
				}
			}
		} break;

		default:
		{
			R04_Manager_Status = R04_MANAGER_UNINIT;
		} break;
	}
}




/**
* @brief        ����ת����
* @param        
* @ref          
* @author       ZFY
* @note         
**/
uint16_t Float2Uint(float val,float min,float max,uint8_t bits)
{

		float span = max - min;
		float offset = min;
		if(val > max) val=max;
		else if(val < min) val= min;
    return (int) ((val-offset)*((float)((1<<bits)-1))/span);
}

/**
* @brief        ����ת����
* @param        
* @ref          
* @author       ZFY
* @note         
**/
float Uint2Float(uint16_t uint, float min,float max,uint8_t bits)
{
    float temp = 0.f;
    if(bits != 0)
        temp = ((float)uint)*(max-min)/((float)((1<<bits)-1)) + min;
    return temp;
}


/**
* @brief        �ؽڵ������
* @param        
* @ref          
* @author       ZFY
* @note         
**/
void Motor_Cmd(uint32_t motor_id, MotorCmdEnum cmd)
{
    uint8_t buff[8]={0};

    //���Ϳ�������
    switch(cmd)
    {
        case RS_ENABLE:    motor_id += (3<<24);   break;
        case RS_DISABLE:   motor_id += (4<<24);   break;
        case RS_ZERO:      motor_id += (6<<24);   buff[0]=1;   break;
        case RS_CLEAR:     motor_id += (4<<24);   buff[0]=1;   break;
    }
    FDCAN1_Send_Msg(buff,8,motor_id);
}



/**
* @brief        �����λ��Ͽ��ƣ�MITģʽ��
* @param        pos:    λ��rad/����
*               vel��   �ٶ�rad/s
*               kp��    λ��PD����ϵ��
*               kd��    λ��PD΢��ϵ��
*               torque������n.m           
* @ref          
* @author       Bling
* @note         
**/
void Motor_Ctrl(uint32_t motor_id,float torque, float MechPosition, float speed, float kp, float kd)
{    
		motor_id += (1<<24); //����˿�ָ��
    uint8_t buff[8];

    //��ֵת��     
    uint16_t position_16   = Float2Uint(MechPosition,P_MIN, P_MAX, 16);     //λ��
    uint16_t velocity_16   = Float2Uint(speed, V_MIN, V_MAX, 16);     //�ٶ�
    uint16_t torque_16     = Float2Uint(torque, T_MIN, T_MAX, 16);     //����
    uint16_t Kp_16       = Float2Uint(kp, KP_MIN,KP_MAX, 16);               //λ��KP����ϵ��
    uint16_t Kd_16       = Float2Uint(kd,KD_MIN,KD_MAX, 16);               //λ��KD����ϵ��
	
	  motor_id+=(torque_16<<8); //���õ��Ť��
    buff[0] = (position_16>>8);
    buff[1] = (position_16);
    buff[2] = (velocity_16>>8);
    buff[3] = (velocity_16);
    buff[4] = (Kp_16>>8);
    buff[5] = (Kp_16);
	  buff[6] = (Kd_16>>8);
    buff[7] = (Kd_16);
    //���Ϳ�������
    FDCAN1_Send_Msg(buff,8,motor_id);
}



/**
* @brief        ����������ݴ���
* @param        
* @ref          
* @author       ZFY
* @note         
**/




void Motor_DataTransform(Rs_Motor *motor,FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxData)
{
    static uint16_t position_16=32768;     //λ��
    static uint16_t velocity_16=32768;     //�ٶ�
    static uint16_t torque_16=32768;     //����
	  static uint16_t temp=32768;     //����
	  if((rx_header->Identifier>>24)==0x02)   //�ж��Ƿ�Ϊ������ص�����֡
		{
		
				uint8_t id = (rx_header->Identifier>>8)& 0x000F;  //���ID
				//������ϴ��룺16/Ƿѹ 17/���� 18/���� 19/�ű������ 20/��ת���� 21/δ�궨 22-23/����ģʽ
				uint16_t status = (rx_header->Identifier >>16)& 0x00FF;
	
				position_16= (rxData[0]<<8)|rxData[1];
        velocity_16= (rxData[2]<<8)|rxData[3];				
		    torque_16  = (rxData[4]<<8)|rxData[5];	
				temp  = (rxData[6]<<8)|rxData[7];	
				
				float posReal = Uint2Float(position_16,P_MIN,P_MAX,16);
				float velReal = Uint2Float(velocity_16,V_MIN,V_MAX,16);
				float torReal = Uint2Float(torque_16,T_MIN,T_MAX,16);
				float tempReal = temp/10.f;		

				if (id < RS_MOTOR_ID_MAX)
				{
					Rs_Motor_List[id].id = id;
					Rs_Motor_List[id].position = posReal;
					Rs_Motor_List[id].speed = Rs_Motor_List[id].speed * 0.5f + velReal * 0.5f;  //�˲�
					Rs_Motor_List[id].torque = torReal;
					Rs_Motor_List[id].status = status;
					Rs_Motor_List[id].temp = tempReal;
					Rs_Motor_Feedback_Valid[id] = 1U;
				}

				if (motor != 0)
				{
					motor->id = id;
					motor->position = posReal;
					motor->speed = motor->speed * 0.5f + velReal * 0.5f;  //�˲�
					motor->torque = torReal;
					motor->status = status;
					motor->temp = tempReal;
				}
		
		}
	
	

   
}

uint8_t RobStride04_HasFeedback(uint8_t motor_id)
{
	if (motor_id >= RS_MOTOR_ID_MAX)
	{
		return 0U;
	}
	return Rs_Motor_Feedback_Valid[motor_id];
}

const Rs_Motor *RobStride04_GetMotor(uint8_t motor_id)
{
	if ((motor_id >= RS_MOTOR_ID_MAX) || (Rs_Motor_Feedback_Valid[motor_id] == 0U))
	{
		return 0;
	}
	return &Rs_Motor_List[motor_id];
}






















