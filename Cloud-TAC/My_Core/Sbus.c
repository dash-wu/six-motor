
#include "Sbus.h"


uint8_t DMA_BUFFER Sbus_buff_[SBUS_SIZE];  //空闲中断数据缓冲区

/********************************************************************************************************
Function Name: Sbus_Init  
Author       : ZFY
Date         : 2025-03-25
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Sbus_Init()
{
	 __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
	 HAL_UART_Receive_DMA(&huart7, (uint8_t*)Sbus_buff_, SBUS_SIZE);     //设置DMA传输，讲串口7的数据搬运到recvive_buff中，
	
}






uint16_t Sbus_CH[SBUS_CH_NUM];  // 遥控器通道值      针对8通道的遥控器
uint8_t __DTCM Sbus_Connect_Flag=0;   //Sbus连接标志位

float __DTCM Sbus_Value[SBUS_CH_NUM]={0.0};

void Sbus_Data_Count(uint8_t *buf)
{
     if((buf[0]==0x19)&&(buf[1]==0x0F))
		 {
				Sbus_CH[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
				Sbus_CH[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
				Sbus_CH[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
				Sbus_CH[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
				Sbus_CH[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
				Sbus_CH[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
				Sbus_CH[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
				Sbus_CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
			 
			 
			  if(Sbus_CH[2]==0x00)
				{
				  Sbus_Connect_Flag=0;
				}else
				{
          Sbus_Connect_Flag=1;
				}				
			 
				
				Sbus_CH[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
				Sbus_CH[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
				Sbus_CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
				Sbus_CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
				Sbus_CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
				Sbus_CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;
				Sbus_CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
				Sbus_CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
				
				for(int i=0;i<SBUS_CH_NUM;i++)
				{
				     Sbus_Value[i]=(float)(Sbus_CH[i]-1024)/784;
				  
				}
				
				
		 }
}







#define SBUS_DATA_SIZE 26  //25个数据 第一个字节的数据为接收到数据的个数
uint8_t __DTCM SBUS_Data[SBUS_DATA_SIZE];    //数据缓冲区
void USAR_UART7_IDLECallback()
{
	  uint8_t Data_Length=0;
    HAL_UART_DMAStop(&huart7);                                                     //停止本次DMA传输	  
    Data_Length  = SBUS_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);   //计算接收到的数据长度
    SBUS_Data[0]=Data_Length;//第一个参数存储接收到的数组个数
		memcpy(SBUS_Data+1,&Sbus_buff_,Data_Length);//将数据全部复制过来				                    //清零接收缓冲区     
		Sbus_Data_Count(SBUS_Data);//功能函数  读取传感器值
    HAL_UART_Receive_DMA(&huart7, (uint8_t*)Sbus_buff_, SBUS_SIZE);                    //重启开始DMA传输 每次BUFFER_SIZE字节数据 
}
























