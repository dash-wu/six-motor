#ifndef MECANUM_WHEEL_H
#define MECANUM_WHEEL_H

#include "main.h"




// 定义轮子速度结构体
typedef struct {
    float front_left;
    float front_right;
    float rear_left;
    float rear_right;
} WheelSpeeds;


extern WheelSpeeds wheel_speeds;

// 麦克纳姆轮底盘控制函数
WheelSpeeds mecanum_control(float forward_speed, float lateral_speed, float angular_speed) ;





#endif //MECANUM_WHEEL_H
