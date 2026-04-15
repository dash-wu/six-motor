#ifndef __WS2812B_H
#define __WS2812B_H

#include "stm32h7xx_hal.h"

#define LED_COUNT 14
#define BIT_COUNT_PER_LED 24
#define DMA_BUFFER_SIZE (LED_COUNT * BIT_COUNT_PER_LED)

extern uint16_t dma_buffer[DMA_BUFFER_SIZE];

void WS2812B_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
void WS2812B_SetPixelColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b, float brightness);
void WS2812B_Show(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
void WS2812B_RainbowEffect(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma,float brightness);



#endif    