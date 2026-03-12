#include "UART.h"



uint8_t DMA_BUFFER Receive_buff_1[BUFFER_SIZE];  //空闲中断数据缓冲区
uint8_t DMA_BUFFER Receive_buff_2[BUFFER_SIZE];  //空闲中断数据缓冲区
uint8_t DMA_BUFFER G_Receive_buff[G_BUFFER_SIZE];  //空闲中断数据缓冲区   4G图传串口信息

/********************************************************************************************************
Function Name: RS485_Init  
Author       : ZFY
Date         : 2025-01-10
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void RS485_Init()
{
	

//	 __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//	 HAL_UART_Receive_DMA(&huart4, (uint8_t*)Receive_buff_1, BUFFER_SIZE);     //设置DMA传输，讲串口4的数据搬运到recvive_buff1中，
//	 HAL_GPIO_WritePin(RS485_1_GPIO_Port,RS485_1_Pin,GPIO_PIN_RESET);	 //拉低 进入接收模式	
//	 __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
//	 HAL_UART_Receive_DMA(&huart5, (uint8_t*)Receive_buff_2, BUFFER_SIZE);     //设置DMA传输，讲串口5的数据搬运到recvive_buff2中，
//	 HAL_GPIO_WritePin(RS485_2_GPIO_Port,RS485_2_Pin,GPIO_PIN_RESET);	 //拉低 进入接收模式
//	
	 __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	 HAL_UART_Receive_DMA(&huart3, (uint8_t*)G_Receive_buff, sizeof(G_Receive_buff));     //串口3 接收 接4G图传模块
	
}



/********************************************************************************************************
Function Name: USAR_UART_IDLECallback 
Author       : ZFY
Date         : 2025-01-10
Description  :
Outputs      : void
Notes        : 空闲中断回调函数
********************************************************************************************************/

#define RX_DATA_SIZE 24 
uint8_t __DTCM Receive_Data_1[RX_DATA_SIZE];    //数据缓冲区
void USAR_UART4_IDLECallback()
{
	  uint8_t Data_Length=0;
    HAL_UART_DMAStop(&huart4);                                                     //停止本次DMA传输	  
    Data_Length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);   //计算接收到的数据长度
    Receive_Data_1[0]=Data_Length;//第一个参数存储接收到的数组个数
		memcpy(Receive_Data_1+1,&Receive_buff_1,Data_Length);//将数据全部复制过来				                    //清零接收缓冲区     
	  Get_Position(Receive_Data_1+1);                                                       //功能函数  读取传感器值
    HAL_UART_Receive_DMA(&huart4, (uint8_t*)Receive_buff_1, BUFFER_SIZE);                    //重启开始DMA传输 每次BUFFER_SIZE字节数据 
}


/********************************************************************************************************
Function Name: USAR_UART_IDLECallback 
Author       : ZFY
Date         : 2025-01-10
Description  :
Outputs      : void
Notes        : 空闲中断回调函数
********************************************************************************************************/
uint8_t __DTCM Receive_Data_2[RX_DATA_SIZE];    //数据缓冲区
void USAR_UART5_IDLECallback()
{
	  uint8_t Data_Length=0;
    HAL_UART_DMAStop(&huart5);                                                     //停止本次DMA传输	  
    Data_Length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);   //计算接收到的数据长度

	  for(int i=0;i<17;i++)
		{
		  if((Receive_buff_2[i]=0xFD)&&(Receive_buff_2[i+1]=0xEE))
			{
			  memcpy(Receive_Data_2,Receive_buff_2+i,16);//将有用数据全部复制过来
				YS_Motor_GetInfo(Receive_Data_2,&Motor_Recvpack);		 //处理电机信息		
				break;
			}
		}

   

	//功能函数
    HAL_UART_Receive_DMA(&huart5, (uint8_t*)Receive_buff_2, BUFFER_SIZE);                    //重启开始DMA传输 每次BUFFER_SIZE字节数据 
	

	
}

/********************************************************************************************************
Function Name: USAR_UART_IDLECallback 
Author       : ZFY
Date         : 2025-01-10
Description  :
Outputs      : void
Notes        : 空闲中断回调函数
********************************************************************************************************/
#define G_DATA_SIZE 20 
uint8_t __DTCM GReceive_Data[G_DATA_SIZE];    //数据缓冲区
void USAR_UART3_IDLECallback()
{
	  uint8_t Data_Length=0;
    HAL_UART_DMAStop(&huart3);                                                     //停止本次DMA传输	  
    Data_Length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);   //计算接收到的数据长度
		memcpy(GReceive_Data,&G_Receive_buff,G_DATA_SIZE);//将数据全部复制过来				                    //清零接收缓冲区		
	  Get_Cmd(GReceive_Data);  //解析4G模块传过来的数据                                                                
    HAL_UART_Receive_DMA(&huart3, (uint8_t*)G_Receive_buff,G_BUFFER_SIZE);                    //重启开始DMA传输 每次BUFFER_SIZE字节数据 
}
/********************************************************************************************************
Function Name: USER_UART_IRQHandler 
Author       : ZFY
Date         : 2025-10-10
Description  :
Outputs      : void
Notes        : 空闲中断
********************************************************************************************************/
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance==UART4)                                   //判断是否是串口4
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart4);                     //清楚空闲中断标志（否则会一直不断进入中断）
            USAR_UART4_IDLECallback();                             //调用中断处理函数
        }
    }else if(huart->Instance==UART5)                                   //判断是否是串口5
		{
        if(RESET != __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart5);                     //清楚空闲中断标志（否则会一直不断进入中断）
            USAR_UART5_IDLECallback();                              //调用中断处理函数
        }		
		
		}else if(huart->Instance==USART3)
		{
        if(RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart3);                     //清楚空闲中断标志（否则会一直不断进入中断）
            USAR_UART3_IDLECallback();                              //调用中断处理函数    
        }				
		
		}else if(huart->Instance==UART7)
		{
        if(RESET != __HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart7);                     //清楚空闲中断标志（否则会一直不断进入中断）
            USAR_UART7_IDLECallback();                              //调用中断处理函数    Sbus
        }				
		
		}else if(huart->Instance==USART1)
		{
        if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //清楚空闲中断标志（否则会一直不断进入中断）
            USAR_UART1_IDLECallback();                              //调用中断处理函数    Ibus
        }				
		
		}
		
		
		
		
		
		
}







/********************************************************************************************************
Function Name: HAL_URS485_Transmit  
Author       : ZFY
Date         : 2025-01-10
Description  : 
Outputs      : void
Notes        : 485发送函数
********************************************************************************************************/
void HAL_URS485_Transmit(UART_HandleTypeDef *huart,uint8_t* _tx_buff,uint8_t _size)
{
	if(huart->Instance == UART4)  //RS485_1
	{
		HAL_GPIO_WritePin(RS485_1_GPIO_Port,RS485_1_Pin,GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(huart,_tx_buff,_size);	   		
	
	}else if(huart->Instance == UART5)//RS485_2
	{
		HAL_GPIO_WritePin(RS485_2_GPIO_Port,RS485_2_Pin,GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(huart,_tx_buff,_size);	
	}
	
}

/********************************************************************************************************
Function Name: HAL_UART_TxCpltCallback  
Author       : ZFY
Date         : 2025-01-10
Description  : 
Outputs      : void
Notes        : 串口发送完成回调函数
********************************************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART4)  //RS485_1
    {       
			HAL_GPIO_WritePin(RS485_1_GPIO_Port,RS485_1_Pin,GPIO_PIN_RESET);	 //串口接收完成后拉低 进入接收模式	
    } else if(huart->Instance == UART5)//RS485_2
		{
		  HAL_GPIO_WritePin(RS485_2_GPIO_Port,RS485_2_Pin,GPIO_PIN_RESET);	 //串口接收完成后拉低 进入接收模式	
		}

}






















