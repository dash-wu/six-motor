#ifndef RM3508_H
#define RM3508_H



#include "main.h"


#define MAX_MOTORS       4       // 最大支持电机数量
#define CAN_ID_TX        0x200   // 发送CAN ID
#define CAN_ID_RX_BASE   0x201   // 接收基ID（ID1~8对应0x201~0x208）

// 电机反馈数据结构
typedef struct {
    int32_t angle;          // 转子角度（多圈累计值）
    int16_t speed;          // 转速（RPM）
	  int16_t speed_last;          // 转速（RPM）
    int16_t torque_current; // 转矩电流
    uint8_t id;             // 电机ID（1-8）
} Motor_Feedback_t;

extern float Motor_Speed[MAX_MOTORS];


void RM3508_Init(void);  //3508电机初始化


void RM3508_filter(FDCAN_HandleTypeDef* fdcanhandle,uint32_t FilterBank_Num,uint32_t _Id);

void RM3508_DataTransform(FDCAN_RxHeaderTypeDef rx_header,uint8_t* _rxdate);

void RM3508_Set_Current(uint8_t motor_id, float _current);   //输入电流值范围为-100 - 100
void RM3508_Send_Current(void);

void Mecanum_wheel_chassis_control(void); // 麦克纳姆轮底盘控制





#endif //RM3508_H
