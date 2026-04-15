
#include "Ibus.h"


uint8_t DMA_BUFFER Ibus_buff[IBUS_SIZE];  //空闲中断数据缓冲区

/********************************************************************************************************
Function Name: Sbus_Init  
Author       : ZFY
Date         : 2025-03-25
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Ibus_Init()
{
	 __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	 HAL_UART_Receive_DMA(&huart1, (uint8_t*)Ibus_buff, IBUS_SIZE);     //设置DMA传输，讲串口7的数据搬运到recvive_buff中，
	
}






uint16_t Ibus_CH[IBUS_CH_NUM];  // 遥控器通道值      针对8通道的遥控器
uint8_t __DTCM Ibus_Connect_Flag=0;   //Sbus连接标志位

float __DTCM Ibus_Remote_Value[IBUS_CH_NUM]={0.0};

void Ibus_Data_Count(uint8_t *buf)
{
     if((buf[1]==0x20)&&(buf[2]==0x40))
		 {
				for(int i=0;i<IBUS_CH_NUM;i++)
				{				   
					 Ibus_CH[i] = (buf[3 + i * 2] | (buf[4 + i * 2] << 8)) & 0x0FFF;
				}	
		 }
		 
		 HAL_UART_Transmit_DMA(&huart1,Ibus_buff,32);  //此行代码只是将接收到遥控器的指令转发给校内赛主板，无其它意义，若采用板间通信，无需采用此方法
		 
		 
		 
}







#define IBUS_DATA_SIZE 26  //25个数据 第一个字节的数据为接收到数据的个数
uint8_t __DTCM IBUS_Data[IBUS_DATA_SIZE];    //数据缓冲区
void USAR_UART1_IDLECallback()
{
	  uint8_t Data_Length=0;
    HAL_UART_DMAStop(&huart1);                                                     //停止本次DMA传输	  
    Data_Length  = IBUS_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);   //计算接收到的数据长度
    IBUS_Data[0]=Data_Length;//第一个参数存储接收到的数组个数
		memcpy(IBUS_Data+1,&Ibus_buff,Data_Length);//将数据全部复制过来				                    //清零接收缓冲区     
		Ibus_Data_Count(IBUS_Data);//功能函数  读取传感器值
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)Ibus_buff, IBUS_SIZE);                    //重启开始DMA传输 每次BUFFER_SIZE字节数据 
}
























