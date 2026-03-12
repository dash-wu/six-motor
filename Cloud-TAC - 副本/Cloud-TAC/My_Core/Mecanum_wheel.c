
#include "Mecanum_wheel.h"
/********************************************************************************************************
Function Name: mecanum_control  
Author       : ZFY
Date         : 2025-04-17
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
WheelSpeeds wheel_speeds;

// 麦克纳姆轮底盘控制函数
WheelSpeeds mecanum_control(float forward_speed, float lateral_speed, float angular_speed) 
{
    // 计算每个轮子的速度
    wheel_speeds.front_left = forward_speed + lateral_speed + angular_speed;
    wheel_speeds.front_right = -(forward_speed - lateral_speed - angular_speed);
    wheel_speeds.rear_left = forward_speed - lateral_speed + angular_speed;
    wheel_speeds.rear_right = -(forward_speed + lateral_speed - angular_speed);
    return wheel_speeds;
}
































