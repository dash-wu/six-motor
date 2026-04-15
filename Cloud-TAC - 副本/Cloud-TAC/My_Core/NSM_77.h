#ifndef NSM_77_H
#define NSM_77_H


#include "main.h"



void NSM_77_Init(void);  //NSM_77毫米波雷达初始化

typedef struct 
{
  uint8_t Objects_ID;
	float Objects_DistLong;
	float Objects_Distlat;
	float Objects_VrelLong;
	float Objects_VrelLat;
	float Theta;
	
}NSM_77;   //

extern float __DTCM Objects_Dis_Min;
extern float __DTCM Objects_Vrel_Max;


#endif //NSM_77_H
