//
// Created by Admin on 2025/01/06.
//
#include "My_Can_Init.h"

FDCAN_TxHeaderTypeDef   TxHeader1;
FDCAN_TxHeaderTypeDef   TxHeader2;
FDCAN_RxHeaderTypeDef   RxHeader1;
FDCAN_RxHeaderTypeDef   RxHeader2;
uint8_t     __DTCM      TxData1[8]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
uint8_t     __DTCM      TxData2[8]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
uint8_t     __DTCM      RxData1[8];
uint8_t     __DTCM      RxData2[8];



/*****************************************************
* CAN滤波器注册函数
*****************************************************/
void My_Can_Filter_Register(FDCAN_HandleTypeDef *_hcan,uint32_t FilterBank_Num,uint32_t _Id)
{
    if(FilterBank_Num>=31)
    {
        return;
    }
    FDCAN_FilterTypeDef  sFilterConfig;
   if(_hcan==&hfdcan1)
   {
       sFilterConfig.IdType = FDCAN_STANDARD_ID;	 /* Configure Rx filter */
       sFilterConfig.FilterIndex = FilterBank_Num;
       sFilterConfig.FilterType = FDCAN_FILTER_MASK;
       sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
       sFilterConfig.FilterID1 = _Id;
       sFilterConfig.FilterID2 = 0x7FF; /* For acceptance, MessageID and FilterID1 must match exactly */
       HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

   } else
   {
       sFilterConfig.IdType = FDCAN_STANDARD_ID;	 /* Configure Rx filter */
       sFilterConfig.FilterIndex = FilterBank_Num;
       sFilterConfig.FilterType = FDCAN_FILTER_MASK;
       sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
       sFilterConfig.FilterID1 = _Id;
       sFilterConfig.FilterID2 = 0x7FF; /* For acceptance, MessageID and FilterID1 must match exactly */
       HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
   }
}
/*****************************************************
* CAN1滤波器初始化
*****************************************************/
void My_CAN1_Filter_Init( )
{
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);/* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  /* Activate Rx FIFO 0 FDCAN_IT_RX_FIFO0_NEW_MESSAGE notification */
    TxHeader1.Identifier = 0x001;  /* Prepare Tx Header */
    TxHeader1.IdType = FDCAN_STANDARD_ID;
    TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader1.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader1.MessageMarker = 0;
    HAL_FDCAN_Start(&hfdcan1);  /* Start the FDCAN module */
}
/*****************************************************
* CAN2滤波器初始化
*****************************************************/
void My_CAN2_Filter_Init( )
{
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);/* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);  /* Activate Rx FIFO 0 FDCAN_IT_RX_FIFO1_NEW_MESSAGE notification */
    TxHeader2.Identifier = 0x001;  /* Prepare Tx Header */
    TxHeader2.IdType = FDCAN_STANDARD_ID;
    TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader2.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader2.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader2.MessageMarker = 0;
    HAL_FDCAN_Start(&hfdcan2);  /* Start the FDCAN module */
}
/***************
 * CAN1发送函数
 ************* */
void My_Can1_Transmit_Message(uint32_t _Id)
{
    TxHeader1.Identifier = _Id;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1, TxData1);
}
/***************
 * CAN2发送函数
 ************* */
void My_Can2_Transmit_Message(uint32_t _Id)
{
    TxHeader2.Identifier = _Id;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&TxHeader2, TxData2);
}


/********************************************************************************************************
Function Name: MY_CAN1_Receive_Mission CAN1接收完成任务处理
Author       : ZFY
Date         : 2025-01-06
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/
__WEAK void MY_CAN1_Receive_Mission(FDCAN_RxHeaderTypeDef* _rxheader,uint8_t* _rx_data)
{
	
				//默认为空
	
}




Rs_Motor Rs_Motor_1;   //创建关节电机1




/***************
 * CAN1接收回调函数
 * 将存储的数据帧数据存储在RxHeader1
 * 将接收的信息存储至RxData1
 ************* */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)  //Can1接收完成回调函数
{
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader1, RxData1);   //*****/* 从RX FIFO0读取数据 */****一定要先取数据
	
	  //MY_CAN1_Receive_Mission(&RxHeader1,RxData1);//CAN1 接收信息任务处理

	  Motor_DataTransform(&Rs_Motor_1,&RxHeader1, RxData1); //读取电机反馈值
	  HAL_GPIO_TogglePin(Led_2_GPIO_Port,Led_2_Pin);
	
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  /* Activate Rx FIFO 0 watermark notification */
}
/***************
 * CAN2接收回调函数
 * 将存储的数据帧数据存储在RxHeader2
 * 将接收的信息存储至RxData2
 ************* */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)  //Can2接收完成回调函数
{
    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &RxHeader2, RxData2);   //*****/* 从RX FIFO1读取数据 */****一定要先取数据
	
	  RM3508_DataTransform(RxHeader2,RxData2);
	
	
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);  /* Activate Rx FIFO 0 FDCAN_IT_RX_FIFO1_NEW_MESSAGE notification */
}








































