#include "NSM_77.h"



#define OB_NUM 10

NSM_77 __DTCM Objects[OB_NUM]; //定义10个目标的信息

float __DTCM Objects_Dis[OB_NUM];
float __DTCM Objects_Vrel[OB_NUM];

float __DTCM Objects_Dis_Min;
float __DTCM Objects_Vrel_Max;

/********************************************************************************************************
Function Name: NSM_77  
Author       : ZFY
Date         : 2025-01-06
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void NSM_77_Init()
{
	
		My_Can_Filter_Register(&hfdcan1,0,0x60B);//注册CAN1通信过滤器     
		My_CAN1_Filter_Init();
}


/********************************************************************************************************
Function Name: Find_Min  
Author       : ZFY
Date         : 2025-01-07
Description  :
Outputs      : void
Notes        :  找出数组中最小的值
********************************************************************************************************/
float Find_Min(float arr[], int len) {
    float min = arr[0];
    for (int i = 1; i < len; i++) {
        if (arr[i] < min) {
            min = arr[i];
        }
    }
    return min;
}

/********************************************************************************************************
Function Name: Find_Max  
Author       : ZFY
Date         : 2025-01-07
Description  :
Outputs      : void
Notes        :  找出数组中最小的值
********************************************************************************************************/
float Find_Max(float arr[], int len) {
    float max = arr[0];
    for (int i = 1; i < len; i++) {
        if (arr[i] > max) {
            max = arr[i];
        }
    }
    return max;
}


/********************************************************************************************************
Function Name: MY_CAN1_Receive_Mission CAN1接收完成任务处理
Author       : ZFY
Date         : 2025-01-06
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/
void MY_CAN1_Receive_Mission(FDCAN_RxHeaderTypeDef* _rxheader,uint8_t* _rx_data)
{
		switch (_rxheader->Identifier) 
			{
        case 0x60B:  //毫米波雷达反馈信息的ID
        {				

          static uint8_t Index=0;
					
					Objects[Index].Objects_ID = _rx_data[0];  //目标ID 
					Objects[Index].Objects_DistLong =  (float) ((int16_t)(_rx_data[1]<<5|(_rx_data[2]>>3)))*0.2f-500.0f; // 目标纵向距离
					Objects[Index].Objects_Distlat  =  (float) ((int16_t)(((_rx_data[2]&0x07)<<8)|_rx_data[3]))*0.2f-204.6f; //目标横向距离
					Objects[Index].Objects_VrelLong =  (float) ((int16_t)( (_rx_data[4]<<2)|(_rx_data[5]>>6)))*0.25f-128.0f; //目标纵向速度
					Objects[Index].Objects_VrelLat  =  (float) ((int16_t)(((_rx_data[5]&0x3F)<<3)|(_rx_data[6]>>5)))*0.25f-64.0f; //目标横向速度
					Objects_Dis[Index]=sqrtf(Objects[Index].Objects_DistLong*Objects[Index].Objects_DistLong+
																		Objects[Index].Objects_Distlat*Objects[Index].Objects_Distlat);					
					Objects[Index].Theta=atanf(Objects[Index].Objects_Distlat/Objects[Index].Objects_DistLong);
					Objects_Vrel[Index]=Objects[Index].Objects_VrelLong*cosf(	Objects[Index].Theta) +	 Objects[Index].Objects_VrelLat*sinf(Objects[Index].Theta);

					Objects_Dis_Min=Find_Min(Objects_Dis,OB_NUM);
					Objects_Vrel_Max=Find_Max(Objects_Vrel,OB_NUM);
					Index++;
					if(Index>=10)
					{
						Index=0;
					}
        };break;

     }

}


























