#include "YS_Motor.h"




//uint8_t DMA_BUFFER Receive_buff[32]={0x00};
//uint8_t __DTCM Receive_Data[16]={0x00};  //测试用



Motor_Send_pack __DTCM Motor_Sendpack;
Motor_recv_pack __DTCM Motor_Recvpack;

Motor_Info __DTCM Motor_1;
Motor_Info __DTCM Motor_2;
Motor_Info __DTCM Motor_3;  //三个电机信息

uint8_t DMA_BUFFER Transmit_buff[17]={0x00};

/********************************************************************************************************
Function Name: CRC 检验  
Author       : ZFY
Date         : 2024-10-31
Description  : 
Outputs      : void
Notes        : 宇树电机初始化
********************************************************************************************************/
//以下CRC_CCITT代码抠自Linux内核
const static __ITCM uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

__STATIC_FORCEINLINE uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}
static uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}



/****************************************************************/

/********************************************************************************************************
Function Name: YS_Motor_Init  
Author       : ZFY
Date         : 2024-10-31
Description  : 
Outputs      : void
Notes        : 宇树电机初始化
********************************************************************************************************/
void YS_Motor_Init()
{

	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	 HAL_UART_Receive_DMA(&huart5, (uint8_t*)Receive_buff_2, BUFFER_SIZE);     //设置DMA传输，讲串口5的数据搬运到recvive_buff2中，
	 HAL_GPIO_WritePin(RS485_2_GPIO_Port,RS485_2_Pin,GPIO_PIN_RESET);	 //拉低 进入接收模式
	
	
}

/********************************************************************************************************
Function Name: YS_Set  
Author       : ZFY
Date         : 2024-10-31
Description  : 
Outputs      : void
Notes        : 宇树电机设置
********************************************************************************************************/
HAL_StatusTypeDef YS_Set(uint8_t motor_id,float p_des,float v_des,float t_ff,float kp,float kd,uint8_t status)
{
	
	
	 Motor_Sendpack.head=0xEEFE;
	 if(status==0x01)
	 {
	  Motor_Sendpack.id_mode=0x10|motor_id; //发送FOC模式指令
	 }else if(status==0x00)
	 {
	  Motor_Sendpack.id_mode=0x00|motor_id; //发送锁定模式指令
	 }else
	 {
     return HAL_ERROR;	 
	 }
	 
		Motor_Sendpack.pset=p_des/(PI*2/32768);
		Motor_Sendpack.wset=v_des/(PI*2/256);
		Motor_Sendpack.tset=t_ff*256;
		Motor_Sendpack.kp=kp*1280;
		Motor_Sendpack.kd=kd*1280;
	  memcpy(Transmit_buff,&Motor_Sendpack.head,15);
		Motor_Sendpack.crc=crc_ccitt(0,Transmit_buff,15);
	  memcpy(Transmit_buff+15,&Motor_Sendpack.crc,2);
	 	 
	  HAL_URS485_Transmit(&huart5,Transmit_buff,sizeof(Transmit_buff));	
	  return HAL_OK;
	
}



/********************************************************************************************************
Function Name: YS_Motor_GetInfo 
Author       : ZFY
Date         : 2023-09-19
Description  :
Outputs      : void
Notes        : 获取电机信息
********************************************************************************************************/

uint8_t ID=0;

void YS_Motor_GetInfo(uint8_t* Receive_Data,Motor_recv_pack* Motor_Info)
{

	memcpy(Motor_Info,Receive_Data,16);//将有用数据全部复制过来
	
	 ID=Motor_Info->id_mode&0x0F;
	
	switch (ID) {
        case 0x00:  //电机Id号 Motor1.id
        {
					Motor_1.pfb=Motor_Info->pfb*(PI*2/32768);
					Motor_1.tfb=Motor_Info->tfb/256;
					Motor_1.wfb=Motor_Info->wfb*(PI*2/256);
					Motor_1.TEMP=Motor_Info->TEMP;
        };break;
        case 0x01:  //电机Id号 Motor2.id
        {
					Motor_2.pfb=(float)Motor_Info->pfb*(PI*2/32768)/6.33;
					Motor_2.tfb=(float)Motor_Info->tfb/256;
					Motor_2.wfb=(float)Motor_Info->wfb*(PI*2/256);
					Motor_2.TEMP=Motor_Info->TEMP;
        };break;
        case 0x02:  //电机Id号 Motor3.id
        {
					Motor_3.pfb=Motor_Info->pfb*(PI*2/32768);
					Motor_3.tfb=Motor_Info->tfb/256;
					Motor_3.wfb=Motor_Info->wfb*(PI*2/256);
					Motor_3.TEMP=Motor_Info->TEMP;
        };break;
      }    
}


///********************************************************************************************************
//Function Name: USER_UART_IRQHandler 
//Author       : ZFY
//Date         : 2023-09-19
//Description  :
//Outputs      : void
//Notes        : 空闲中断
//********************************************************************************************************/
//void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
//{
//	  uint8_t Data_Length=0;
//    HAL_UART_DMAStop(&huart2);                                                     //停止本次DMA传输	  
//    Data_Length  = sizeof(Receive_buff) - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   //计算接收到的数据长度
////    Receive_Data[0]=Data_Length;//第一个参数存储接收到的数组个数
//	 
//	  for(int i=0;i<17;i++)
//		{
//		  if((Receive_buff[i]=0xFD)&&(Receive_buff[i+1]=0xEE))
//			{
//			  memcpy(Receive_Data,Receive_buff,16);//将有用数据全部复制过来
//				YS_Motor_GetInfo(Receive_Data,&Motor_Recvpack);		 //处理电机信息		
//				break;
//			}
//		}
//	  HAL_GPIO_TogglePin(Led2_GPIO_Port,Led2_Pin);   //接收到一帧数据灯闪烁一次
//	  HAL_UART_Receive_DMA(&huart2, (uint8_t*)Receive_buff, sizeof(Receive_buff));     //设置DMA传输，讲串口2的数据搬运到recvive_buff中，
//}





















