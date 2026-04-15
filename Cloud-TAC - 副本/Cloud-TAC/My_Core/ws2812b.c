#include "ws2812b.h"
#include "string.h"


uint16_t dma_buffer[DMA_BUFFER_SIZE];

// 宏定义0和1的PWM占空比
#define WS2812B_T0H 1  // 逻辑0高电平持续时间对应的计数值
#define WS2812B_T1H 2  // 逻辑1高电平持续时间对应的计数值
#define WS2812B_PERIOD 3  // 一个位周期的计数值

// 初始化WS2812B
void WS2812B_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma) {
    HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1, (uint32_t *)dma_buffer, DMA_BUFFER_SIZE);
    HAL_Delay(1);
}

//// 设置单个像素的颜色
//void WS2812B_SetPixelColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
//    uint16_t offset = index * BIT_COUNT_PER_LED;
//    for (int i = 0; i < 8; i++) {
//        dma_buffer[offset + i] = (g & (1 << (7 - i))) ? WS2812B_T1H : WS2812B_T0H;
//    }
//    for (int i = 0; i < 8; i++) {
//        dma_buffer[offset + 8 + i] = (r & (1 << (7 - i))) ? WS2812B_T1H : WS2812B_T0H;
//    }
//    for (int i = 0; i < 8; i++) {
//        dma_buffer[offset + 16 + i] = (b & (1 << (7 - i))) ? WS2812B_T1H : WS2812B_T0H;
//    }
//}


// 设置单个像素的颜色，并可调节亮度
void WS2812B_SetPixelColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b, float brightness) {
    r = (uint8_t)(r * brightness);
    g = (uint8_t)(g * brightness);
    b = (uint8_t)(b * brightness);

    uint16_t offset = index * BIT_COUNT_PER_LED;
    for (int i = 0; i < 8; i++) {
        dma_buffer[offset + i] = (g & (1 << (7 - i))) ? WS2812B_T1H : WS2812B_T0H;
    }
    for (int i = 0; i < 8; i++) {
        dma_buffer[offset + 8 + i] = (r & (1 << (7 - i))) ? WS2812B_T1H : WS2812B_T0H;
    }
    for (int i = 0; i < 8; i++) {
        dma_buffer[offset + 16 + i] = (b & (1 << (7 - i))) ? WS2812B_T1H : WS2812B_T0H;
    }
}



// 显示颜色
void WS2812B_Show(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma) {
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1, (uint32_t *)dma_buffer, DMA_BUFFER_SIZE);
}

// 彩虹颜色转换函数
void HSVtoRGB(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
    int i;
    float f, p, q, t;
    if (s == 0) {
        *r = *g = *b = (uint8_t)(v * 255);
        return;
    }
    h /= 60;
    i = (int)h;
    f = h - i;
    p = v * (1 - s);
    q = v * (1 - s * f);
    t = v * (1 - s * (1 - f));
    switch (i) {
        case 0:
            *r = (uint8_t)(v * 255);
            *g = (uint8_t)(t * 255);
            *b = (uint8_t)(p * 255);
            break;
        case 1:
            *r = (uint8_t)(q * 255);
            *g = (uint8_t)(v * 255);
            *b = (uint8_t)(p * 255);
            break;
        case 2:
            *r = (uint8_t)(p * 255);
            *g = (uint8_t)(v * 255);
            *b = (uint8_t)(t * 255);
            break;
        case 3:
            *r = (uint8_t)(p * 255);
            *g = (uint8_t)(q * 255);
            *b = (uint8_t)(v * 255);
            break;
        case 4:
            *r = (uint8_t)(t * 255);
            *g = (uint8_t)(p * 255);
            *b = (uint8_t)(v * 255);
            break;
        default:
            *r = (uint8_t)(v * 255);
            *g = (uint8_t)(p * 255);
            *b = (uint8_t)(q * 255);
            break;
    }
}

//// 彩虹灯效果
//void WS2812B_RainbowEffect(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma) {
//    for (int j = 0; j < 360; j++) {
//        for (int i = 0; i < LED_COUNT; i++) {
//            uint8_t r, g, b;
//            HSVtoRGB((j + i * 45) % 360, 1.0, 1.0, &r, &g, &b);
//            WS2812B_SetPixelColor(i, r, g, b);
//        }
//        WS2812B_Show(htim, hdma);
//        HAL_Delay(20);
//    }
//}

// 彩虹灯效果，并可调节亮度
void WS2812B_RainbowEffect(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma, float brightness) {
    for (int j = 0; j < 360; j++) {
        for (int i = 0; i < LED_COUNT; i++) {
            uint8_t r, g, b;
            HSVtoRGB((j + i * 45) % 360, 1.0, 1.0, &r, &g, &b);
            WS2812B_SetPixelColor(i, r, g, b, brightness);
        }
        WS2812B_Show(htim, hdma);
        HAL_Delay(20);
    }
}


