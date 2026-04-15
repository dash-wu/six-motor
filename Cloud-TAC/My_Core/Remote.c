#include "Remote.h"




/********************************************************************************************************
Function Name: Remote_Init   遥控器串口接收初始化
Author       : ZFY
Date         : 2024-12-11
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/
void Remote_Init()
{

	HAL_TIM_Base_Start_IT(&htim12);  //开启定时器12  输入捕捉
  HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_2); //定时器12
	
	HAL_TIM_Base_Start_IT(&htim1);  //开启定时器8     输入捕捉
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2); //定时器8
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4); //定时器8  
	
	
}


/********************************************************************************************************
Function Name: HAL_TIM_IC_CaptureCallback  定时器输入捕捉回调函数
Author       : ZFY
Date         : 2024-12-11
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/

uint16_t __DTCM Tim1_Ch1_PulseWidth = 0;      // 脉冲宽度
uint16_t __DTCM Tim1_Ch2_PulseWidth = 0;      // 脉冲宽度
uint16_t __DTCM Tim1_Ch3_PulseWidth = 0;      // 脉冲宽度
uint16_t __DTCM Tim1_Ch4_PulseWidth = 0;      // 脉冲宽度
uint32_t __DTCM Tim12_Ch1_PulseWidth = 0;      // 脉冲宽度
uint32_t __DTCM Tim12_Ch2_PulseWidth = 0;      // 脉冲宽度

uint16_t __DTCM Channel_Value[6]={0};

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	

  if(htim->Instance == TIM1)
  {
	
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // 确认是通道1中断
				{	

						static uint32_t CH1_RisingEdge = 0;   // 上升沿捕获值
						static uint32_t CH1_FallingEdge = 0;  // 下降沿捕获值
						static uint8_t IsRisingEdge_1 = 1; // 标志位，用于区分上升沿和下降沿
						if (IsRisingEdge_1) // 上升沿捕获
						{
								CH1_RisingEdge = htim->Instance->CCR1;
								if(HAL_GPIO_ReadPin(TIM1_CH1_GPIO_Port,TIM1_CH1_Pin)==0x01)
								{
								  IsRisingEdge_1 = 0; // 切换到下降沿捕获
								}
						}
						else // 下降沿捕获
						{
							  CH1_FallingEdge = htim->Instance->CCR1;
								// 计算脉冲宽度
								if (CH1_FallingEdge >= CH1_RisingEdge)
								{
										Tim1_Ch1_PulseWidth = CH1_FallingEdge - CH1_RisingEdge;
								}
								else
								{
										Tim1_Ch1_PulseWidth = (htim->Instance->ARR - CH1_RisingEdge) + CH1_FallingEdge + 1;
								}
								IsRisingEdge_1 = 1; // 切换回上升沿捕获
						}
				}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // 确认是通道2中断
				{	
						static uint32_t CH2_RisingEdge = 0;   // 上升沿捕获值
						static uint32_t CH2_FallingEdge = 0;  // 下降沿捕获值
						static uint8_t IsRisingEdge_2 = 1; // 标志位，用于区分上升沿和下降沿

						if (IsRisingEdge_2) // 上升沿捕获
						{
								CH2_RisingEdge = htim->Instance->CCR2;
								if(HAL_GPIO_ReadPin(TIM1_CH2_GPIO_Port,TIM1_CH2_Pin)==0x01)
								{
								  IsRisingEdge_2 = 0; // 切换到下降沿捕获
								}
						}
						else // 下降沿捕获
						{
							  CH2_FallingEdge = htim->Instance->CCR2;
								// 计算脉冲宽度
								if (CH2_FallingEdge >= CH2_RisingEdge)
								{
										Tim1_Ch2_PulseWidth = CH2_FallingEdge - CH2_RisingEdge;
								}
								else
								{
										Tim1_Ch2_PulseWidth = (htim->Instance->ARR - CH2_RisingEdge) + CH2_FallingEdge + 1;
								}
								IsRisingEdge_2 = 1; // 切换回上升沿捕获
						}					
				}else	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) // 确认是通道3中断
				{	
						static uint32_t CH3_RisingEdge = 0;   // 上升沿捕获值
						static uint32_t CH3_FallingEdge = 0;  // 下降沿捕获值
						static uint8_t IsRisingEdge_3 = 1; // 标志位，用于区分上升沿和下降沿

						if (IsRisingEdge_3) // 上升沿捕获
						{
								CH3_RisingEdge = htim->Instance->CCR3;
								if(HAL_GPIO_ReadPin(TIM1_CH3_GPIO_Port,TIM1_CH3_Pin)==0x01)
								{
								  IsRisingEdge_3 = 0; // 切换到下降沿捕获
								}
						}
						else // 下降沿捕获
						{
							  CH3_FallingEdge = htim->Instance->CCR3;
								// 计算脉冲宽度
								if (CH3_FallingEdge >= CH3_RisingEdge)
								{
										Tim1_Ch3_PulseWidth = CH3_FallingEdge - CH3_RisingEdge;
								}
								else
								{
										Tim1_Ch2_PulseWidth = (htim->Instance->ARR - CH3_RisingEdge) + CH3_FallingEdge + 1;
								}
								IsRisingEdge_3 = 1; // 切换回上升沿捕获
						}		
				}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // 确认是通道2中断
				{	
						static uint32_t CH4_RisingEdge = 0;   // 上升沿捕获值
						static uint32_t CH4_FallingEdge = 0;  // 下降沿捕获值
						static uint8_t IsRisingEdge_4 = 1; // 标志位，用于区分上升沿和下降沿

						if (IsRisingEdge_4) // 上升沿捕获
						{
								CH4_RisingEdge = htim->Instance->CCR4;
								if(HAL_GPIO_ReadPin(TIM1_CH4_GPIO_Port,TIM1_CH4_Pin)==0x01)
								{
								  IsRisingEdge_4 = 0; // 切换到下降沿捕获
								}
						}
						else // 下降沿捕获
						{
							  CH4_FallingEdge = htim->Instance->CCR4;
								// 计算脉冲宽度
								if (CH4_FallingEdge >= CH4_RisingEdge)
								{
										Tim1_Ch4_PulseWidth = CH4_FallingEdge - CH4_RisingEdge;
								}
								else
								{
										Tim1_Ch4_PulseWidth = (htim->Instance->ARR - CH4_RisingEdge) + CH4_FallingEdge + 1;
								}
								IsRisingEdge_4 = 1; // 切换回上升沿捕获
						}				
				}							
		


		
		
  }else if(htim->Instance == TIM12)
			
	{
		
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // 确认是通道1中断
				{										static uint32_t CH1_RisingEdge = 0;   // 上升沿捕获值
						static uint32_t CH1_FallingEdge = 0;  // 下降沿捕获值
						static uint8_t IsRisingEdge_1 = 1; // 标志位，用于区分上升沿和下降沿
						if (IsRisingEdge_1) // 上升沿捕获
						{
								CH1_RisingEdge = htim->Instance->CCR1;
								if(HAL_GPIO_ReadPin(TIM12_CH1_GPIO_Port,TIM12_CH1_Pin)==0x01)
								{
								  IsRisingEdge_1 = 0; // 切换到下降沿捕获
								}
						}
						else // 下降沿捕获
						{
							  CH1_FallingEdge = htim->Instance->CCR1;
								// 计算脉冲宽度
								if (CH1_FallingEdge >= CH1_RisingEdge)
								{
										Tim12_Ch1_PulseWidth = CH1_FallingEdge - CH1_RisingEdge;
								}
								else
								{
										Tim12_Ch1_PulseWidth = (htim->Instance->ARR - CH1_RisingEdge) + CH1_FallingEdge + 1;
								}
								IsRisingEdge_1 = 1; // 切换回上升沿捕获
						}
				}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // 确认是通道2中断
				{	
						static uint32_t CH2_RisingEdge = 0;   // 上升沿捕获值
						static uint32_t CH2_FallingEdge = 0;  // 下降沿捕获值
						static uint8_t IsRisingEdge_2 = 1; // 标志位，用于区分上升沿和下降沿

						if (IsRisingEdge_2) // 上升沿捕获
						{
								CH2_RisingEdge = htim->Instance->CCR2;
								if(HAL_GPIO_ReadPin(TIM12_CH2_GPIO_Port,TIM12_CH2_Pin)==0x01)
								{
								  IsRisingEdge_2 = 0; // 切换到下降沿捕获
								}
						}
						else // 下降沿捕获
						{
							  CH2_FallingEdge = htim->Instance->CCR2;
								// 计算脉冲宽度
								if (CH2_FallingEdge >= CH2_RisingEdge)
								{
										Tim12_Ch2_PulseWidth = CH2_FallingEdge - CH2_RisingEdge;
								}
								else
								{
										Tim12_Ch2_PulseWidth = (htim->Instance->ARR - CH2_RisingEdge) + CH2_FallingEdge + 1;
								}
								IsRisingEdge_2 = 1; // 切换回上升沿捕获
						}					
				}
		
	
	}


Channel_Value[0]=Channel_Value[0]*LPF_RATE+Tim1_Ch1_PulseWidth*(1.0f-LPF_RATE);
Channel_Value[1]=Channel_Value[1]*LPF_RATE+Tim1_Ch2_PulseWidth*(1.0f-LPF_RATE);
Channel_Value[2]=Channel_Value[2]*LPF_RATE+Tim1_Ch3_PulseWidth*(1.0f-LPF_RATE);
Channel_Value[3]=Channel_Value[3]*LPF_RATE+Tim1_Ch4_PulseWidth*(1.0f-LPF_RATE);
Channel_Value[4]=Channel_Value[4]*LPF_RATE+Tim12_Ch1_PulseWidth*(1.0f-LPF_RATE);
Channel_Value[5]=Channel_Value[5]*LPF_RATE+Tim12_Ch2_PulseWidth*(1.0f-LPF_RATE);

	
	
for(int i=0;i<6;i++)
{
	if(Channel_Value[i]<500)
	{
	  Channel_Value[i]=500;
	}else if(Channel_Value[i]>2500)
	{
	  Channel_Value[i]=2500;
	}
}
	
	
	
	

}



/********************************************************************************************************
Function Name: Get_Cmd  
Author       : ZFY
Date         : 2025-01-10
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/



void Get_Cmd(uint8_t* _rx_data)   //获取4G图传遥控端发送来的指令
{
	uint16_t CRC_OUT=0;
	CRC_OUT=do_crc_table(_rx_data,12); //crc校验
	
	
	if(((CRC_OUT>>8)==_rx_data[12])&&((CRC_OUT&0x00FF)==_rx_data[13])) //CRC校验通过
	{
    memcpy(Channel_Value,_rx_data,12); //将数据通道里的数都复制过来		
		for(int i=0;i<6;i++)
		{
			if(Channel_Value[i]<500)
			{
				Channel_Value[i]=500;
			}else if(Channel_Value[i]>2500)
			{
				Channel_Value[i]=2500;
			}
		}
	
		
		

	}
	
	
}




















