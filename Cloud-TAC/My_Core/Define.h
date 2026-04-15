#ifndef DEFINE_H
#define DEFINE_H

#include "stm32h7xx_hal.h"

#define DMA_BUFFER __attribute__((section(".RAM_D1")))  //宏定义
#define __DTCM     __attribute__((section(".DTCM")))    //宏定义
#define __ITCM   __attribute__((section(".ITCM")))


//测试板子的时候Debug一个变量的内存地址





#endif //DEFINE_H
