#ifndef IBUS_H
#define IBUS_H





#include "main.h"


#define  IBUS_SIZE  (40)
#define IBUS_CH_NUM 14

extern DMA_HandleTypeDef hdma_usart1_rx;
void Ibus_Init(void);
void USAR_UART1_IDLECallback(void);

extern uint16_t Ibus_CH[IBUS_CH_NUM];  // 遥控器通道值      针对8通道的遥控器


#endif //IBUS_H
