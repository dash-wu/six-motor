//
// Created by Admin on 2025/01/06.
//

#ifndef H743_CAN_INIT_H
#define H743_CAN_INIT_H
#ifdef __cplusplus
extern "C"{

#endif
/*----------------------C------------------------------*/
#include "main.h"
#include "gpio.h"  //此处添加其他的C头文件
#include "fdcan.h"
#include "RM3508.h"


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern FDCAN_TxHeaderTypeDef   TxHeader1;
extern FDCAN_RxHeaderTypeDef   RxHeader1;
extern FDCAN_RxHeaderTypeDef   RxHeader2;
extern FDCAN_TxHeaderTypeDef   TxHeader2;
extern uint8_t         __DTCM      TxData1[8];
extern uint8_t         __DTCM      TxData2[8];
extern uint8_t         __DTCM      RxData1[8];
extern uint8_t         __DTCM      RxData2[8];
extern void My_CAN1_Filter_Init(void);
extern void My_CAN2_Filter_Init(void);
extern void My_Can_Filter_Register(FDCAN_HandleTypeDef *_hcan,uint32_t FilterBank_Num,uint32_t _Id);//注册过滤器
extern void My_Can1_Transmit_Message(uint32_t _Id);
extern void My_Can2_Transmit_Message(uint32_t _Id);

void MY_CAN1_Receive_Mission(FDCAN_RxHeaderTypeDef* _rxheader,uint8_t* _rx_data); 








#ifdef __cplusplus

}
/*----------------------C++------------------------------*/





#endif





#endif
