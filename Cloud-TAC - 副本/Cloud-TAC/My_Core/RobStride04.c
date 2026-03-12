
#include "RobStride04.h"


FDCAN_FilterTypeDef R04_sFilterConfig;
FDCAN_TxHeaderTypeDef R04_TxHeader;
FDCAN_RxHeaderTypeDef R04_RxHeader;

void R04_filter(FDCAN_HandleTypeDef* fdcanhandle,uint32_t FilterBank_Num)
{
	
	R04_sFilterConfig.IdType=FDCAN_EXTENDED_ID;//��չID
	R04_sFilterConfig.FilterIndex=FilterBank_Num;//�˲�������   
	R04_sFilterConfig.FilterType=FDCAN_FILTER_RANGE;      //�˲�������
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

	 
	
	if(HAL_FDCAN_ConfigFilter(fdcanhandle,&R04_sFilterConfig)!=HAL_OK)//�˲�����ʼ��
	HAL_FDCAN_ConfigGlobalFilter(fdcanhandle,FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);//FDCAN_ACCEPT_IN_RX_FIFO0
	HAL_FDCAN_ActivateNotification(fdcanhandle,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	HAL_FDCAN_Start(fdcanhandle);                               //����FDCAN

}



uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len,uint32_t CAN_ID)
{	
	R04_TxHeader.Identifier=CAN_ID;                           //32λID
	R04_TxHeader.DataLength=len<<16;                            //���ݳ���
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&R04_TxHeader,msg)!=HAL_OK) {
		return 1;}//����}
    return 0;	
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
 * @brief  �򻯵ĵ��λ�ÿ��ƽӿڣ���F1�ɹ���Motor_Control���ݣ�
 * @param  motor_id: ���ID (1-20)
 * @param  angle_deg: Ŀ��Ƕȣ�������0-360�ȣ�>180��ӳ�䵽-180~180��
 * @param  speed_rad_s: �ٶ����ƣ�rad/s��
 * @retval 0=�ɹ���1=������Ч
 */
uint8_t Motor_Control(uint8_t motor_id, float angle_deg, float speed_rad_s)
{
	if (motor_id == 0U)
	{
		return 1U;
	}

	/* ��0~360��ӳ�䵽-180~180�ȣ���ת���� */
	float angle_normalized = angle_deg;
	if (angle_normalized > 180.0f)
	{
		angle_normalized -= 360.0f;
	}
	float angle_rad = angle_normalized * 3.14159265359f / 180.0f;

	/* �޷���Э�������Ļ�е�Ƕȷ�Χ */
	if (angle_rad < P_MIN) angle_rad = P_MIN;
	if (angle_rad > P_MAX) angle_rad = P_MAX;

	/* �ٶ��޷� */
	if (speed_rad_s < V_MIN) speed_rad_s = V_MIN;
	if (speed_rad_s > V_MAX) speed_rad_s = V_MAX;

	/* ʹ��һ��Ĭ�ϵ���λ��Ͽ��Ʋ������ɸ�����Ҫ���ⲿ���� */
	const float default_kp = 0.5f;
	const float default_kd = 0.02f;
	const float default_torque = 0.0f;

	Motor_Ctrl((uint32_t)motor_id, default_torque, angle_rad, speed_rad_s, default_kp, default_kd);
	HAL_Delay(5);

	return 0U;
}

/**
 * @brief  ����ǰ��еλ�ñ���Ϊ��㣨��F1�ɹ���Motor_SetControl���ݣ�
 * @param  motor_id: ���ID (1-20)
 * @retval 0=�ɹ���1=������Ч
 */
uint8_t Motor_SetControl(uint8_t motor_id)
{
	if (motor_id == 0U)
	{
		return 1U;
	}

	/* ͨ��RS_ZERO����ѵ�ǰ��еλ�ñ���Ϊ��� */
	Motor_Cmd((uint32_t)motor_id, RS_ZERO);

	/* �����һЩʱ����ɱ������ */
	HAL_Delay(200);

	return 0U;
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
			
				motor->id=id;
				motor->position=posReal;
				motor->speed=motor->speed*0.5f+velReal*0.5f;  //�˲�
				motor->torque=torReal;
				motor->status=status;
				motor->temp=tempReal;
		
		}
	
	

   
}






















