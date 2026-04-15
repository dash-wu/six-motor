#ifndef UART_H
#define UART_H


#include "main.h"
#include "usart.h"
#include "Sbus.h"
#include "YS_Motor.h"

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;



//#define Rx_Size 12
//extern uint8_t Rx_Buffer_1[Rx_Size];
//extern uint8_t Rx_Data_1[Rx_Size];
//extern uint8_t Rx_Buffer_2[Rx_Size];
//extern uint8_t Rx_Data_2[Rx_Size];

#define  BUFFER_SIZE  (50)
extern uint8_t DMA_BUFFER Receive_buff_1[BUFFER_SIZE];
extern uint8_t DMA_BUFFER Receive_buff_2[BUFFER_SIZE];

#define  G_BUFFER_SIZE  (40)


void RS485_Init(void); //RS485通信初始化
void USAR_UART4_IDLECallback();//串口4空闲回调函数
void USAR_UART5_IDLECallback();//串口5空闲回调函数
void USAR_UART3_IDLECallback();//串口3空闲回调函数


void USER_UART_IRQHandler(UART_HandleTypeDef *huart);//空闲中断


void HAL_URS485_Transmit(UART_HandleTypeDef *huart,uint8_t* _tx_buff,uint8_t _size); //485发送函数


#endif //UART_H
