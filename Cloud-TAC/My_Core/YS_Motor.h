#ifndef YS_MOTOR_H
#define YS_MOTOR_H

#include "App.h"
#include "string.h"



//#define PI 3.14159265359f



typedef struct __PACKED send_pack{
	
	uint16_t head;//包头(2Byte)
	uint8_t id_mode;//模式设置(1Byte)
	int16_t tset;//期望电机转矩
	int16_t wset;//期望电机转速
	int32_t pset;//期望电机位置
	int16_t kp;//位置误差比例系数
	int16_t kd;//速度误差比例系数
	uint16_t crc;//CRC16-CCITT(2Byte)
}Motor_Send_pack;

typedef struct __PACKED recv_pack{
	
	uint16_t head;//包头(2Byte)
	uint8_t id_mode;//模式设置(0.5Byte)
	int16_t tfb;//反馈电机转矩
	int16_t wfb;//反馈电机转速
	int32_t pfb;//反馈电机位置
	int8_t TEMP;
	uint8_t unused[2];
	uint16_t crc;//CRC16-CCITT(2Byte)
}Motor_recv_pack;

typedef struct  motor_pack{
	
	uint8_t id;//模式设置(0.5Byte)
	uint8_t mode;//电机模式(0.5Byte)
	float tfb;//反馈电机转矩
	float wfb;//反馈电机转速
	float pfb;//反馈电机位置
	int8_t TEMP;

}Motor_Info;



__STATIC_FORCEINLINE uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);


extern Motor_Send_pack __DTCM Motor_Sendpack;
extern Motor_recv_pack __DTCM Motor_Recvpack;

extern Motor_Info __DTCM Motor_1;
extern Motor_Info __DTCM Motor_2;
extern Motor_Info __DTCM Motor_3;  //三个电机信息


void YS_Motor_Init();//宇树电机初始化
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
HAL_StatusTypeDef YS_Set(uint8_t motor_id,float p_des,float v_des,float t_ff,float kp,float kd,uint8_t status);
void YS_Motor_GetInfo(uint8_t* Receive_Data,Motor_recv_pack* Motor_Info);




#endif //YS_MOTOR_H
