#ifndef ROBSTRIDE04_H
#define ROBSTRIDE04_H

#include "main.h"
//extern FDCAN_FilterTypeDef R04_sFilterConfig;
//extern FDCAN_TxHeaderTypeDef R04_TxHeader;
//extern FDCAN_RxHeaderTypeDef R04_RxHeader;





#define P_MIN -12.57f //0.4.0.5��֮ǰΪ12.5��֮��Ϊ12.57
#define P_MAX 12.57f //0.4.0.5��֮ǰΪ12.5��֮��Ϊ12.57
#define V_MIN -15.0f
#define V_MAX 15.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -120.0f
#define T_MAX 120.0f
#define RS_MOTOR_ID_MAX 16U


/**
* @brief    �������ö��
**/
typedef enum 
{
    RS_ENABLE = 0,     //ʹ��
    RS_DISABLE,        //ʧ��
    RS_CLEAR,          //�������
    RS_ZERO,           //����Ϊ���
}MotorCmdEnum;



/*Functions------------------------------------------------------------------*/
uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len,uint32_t CAN_ID);
void R04_filter(FDCAN_HandleTypeDef* fdcanhandle,uint32_t FilterBank_Num);

void Motor_Cmd(uint32_t motor_id, MotorCmdEnum cmd);
void Motor_Ctrl(uint32_t motor_id,float torque, float MechPosition, float speed, float kp, float kd); //���õ��


typedef struct 
{
	uint8_t id;
	float position,speed,torque,temp;
	uint16_t status;
}Rs_Motor;   //����TMOTOR����ṹ��

extern Rs_Motor Rs_Motor_List[RS_MOTOR_ID_MAX];
extern uint8_t Rs_Motor_Feedback_Valid[RS_MOTOR_ID_MAX];

void RobStride04_Manager_Init(const uint8_t *motor_ids, uint8_t motor_count);
void RobStride04_Manager_Task1ms(void);
uint8_t RobStride04_IsReady(void);
HAL_StatusTypeDef RobStride04_Set(uint8_t motor_id, float p_des, float v_des, float t_ff, float kp, float kd, uint8_t status);

void Motor_DataTransform(Rs_Motor *motor,FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxData);
uint8_t RobStride04_HasFeedback(uint8_t motor_id);
const Rs_Motor *RobStride04_GetMotor(uint8_t motor_id);







#endif //ROBSTRIDE04_H
