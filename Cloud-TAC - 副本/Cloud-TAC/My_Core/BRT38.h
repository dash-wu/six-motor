#ifndef BRT38_H
#define BRT38_H





#include "main.h"
#include "UART.h"
#include "CRC.h"


#define PI 3.14159265f

extern float BRT38_Position;


void BRT38_Init(void); //编码器初始化
void Position_Read(uint8_t Dev_add,uint16_t _add,uint16_t _num);  //编码位置读取指令
void Get_Position(uint8_t* _rx_data);//解析拉线传感器的值



#endif //BRT38_H
