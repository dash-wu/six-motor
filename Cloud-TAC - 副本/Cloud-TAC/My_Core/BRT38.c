
#include "BRT38.h"


float BRT38_Position=0.0f;

/********************************************************************************************************
Function Name: BRT38_Init  
Author       : ZFY
Date         : 2025-01-10
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void BRT38_Init()
{

	 __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	 HAL_UART_Receive_DMA(&huart4, (uint8_t*)Receive_buff_1, BUFFER_SIZE);     //设置DMA传输，讲串口4的数据搬运到recvive_buff1中，
	 HAL_GPIO_WritePin(RS485_1_GPIO_Port,RS485_1_Pin,GPIO_PIN_RESET);	 //拉低 进入接收模式	
	
}



/********************************************************************************************************
Function Name: Position_Read  
Author       : 
Date         : 
Description  :
Outputs      : void
Notes        : 读取采集卡的值 接在串口2上
********************************************************************************************************/
uint8_t DMA_BUFFER Tx_Buff[8];
void Position_Read(uint8_t Dev_add,uint16_t _add,uint16_t _num)
{
//  uint8_t Tx_Buff[8];
	uint16_t CRC_OUT=0;
	Tx_Buff[0]=Dev_add;
	Tx_Buff[1]=0x03;
	Tx_Buff[2]=(_add&0xFF00)>>8;
	Tx_Buff[3]=(_add&0x00FF);
	Tx_Buff[4]=(_num&0xFF00)>>8;
	Tx_Buff[5]=(_num&0x00FF);	
  CRC_OUT=do_crc_table(Tx_Buff,6);//CRC校验
	Tx_Buff[6]=(CRC_OUT&0x00FF);	    //低位在前
	Tx_Buff[7]=(CRC_OUT&0xFF00)>>8;   //高位在后
	
	HAL_URS485_Transmit(&huart4,Tx_Buff,sizeof(Tx_Buff));//数据发送
}

/********************************************************************************************************
Function Name: Position_Read  
Author       : 
Date         : 
Description  :
Outputs      : void
Notes        :获取拉线传感器的值
********************************************************************************************************/
uint16_t CRC_OUT_BRT=0;


void Get_Position(uint8_t* _rx_data)
{

	
	uint32_t P_BUFF=0;
	CRC_OUT_BRT=do_crc_table(_rx_data,7); //crc校验
	if(((CRC_OUT_BRT>>8)==_rx_data[8])&&((CRC_OUT_BRT&0x00FF)==_rx_data[7])) //CRC校验通过
	{
			P_BUFF=(_rx_data[3]<<24)|(_rx_data[4]<<16)|(_rx_data[5]<<8)|(_rx_data[6]);
      BRT38_Position=P_BUFF*0.098f;
			if(BRT38_Position>=600.0f)
			{
			    BRT38_Position=0.0f;
			}
	}
}





//中位223mm   左167  -56      右 295   72





















