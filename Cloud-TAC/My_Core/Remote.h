//
// Created by Admin on 2023/3/9.
//

#ifndef Remote_H
#define Remote_H

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "App.h"
#include "Define.h"


#define LPF_RATE  0.8f



void Remote_Init(void);  //遥控器初始化

void Get_Cmd(uint8_t* _rx_data);   //获取4G图传遥控端发送来的指令

extern uint16_t __DTCM Channel_Value[6];  //遥控器每个通道的值



#endif //Remote_H
