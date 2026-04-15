/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "App.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_3_Pin GPIO_PIN_2
#define Led_3_GPIO_Port GPIOE
#define Step_D2_Pin GPIO_PIN_3
#define Step_D2_GPIO_Port GPIOE
#define Step_D1_Pin GPIO_PIN_4
#define Step_D1_GPIO_Port GPIOE
#define Step_P1_Pin GPIO_PIN_5
#define Step_P1_GPIO_Port GPIOE
#define Step_P2_Pin GPIO_PIN_6
#define Step_P2_GPIO_Port GPIOE
#define Led_4_Pin GPIO_PIN_13
#define Led_4_GPIO_Port GPIOC
#define IN_1_Pin GPIO_PIN_0
#define IN_1_GPIO_Port GPIOC
#define IN_2_Pin GPIO_PIN_1
#define IN_2_GPIO_Port GPIOC
#define IN_3_Pin GPIO_PIN_2
#define IN_3_GPIO_Port GPIOC
#define IN_4_Pin GPIO_PIN_3
#define IN_4_GPIO_Port GPIOC
#define PWM_OUT_5_Pin GPIO_PIN_0
#define PWM_OUT_5_GPIO_Port GPIOA
#define PWM_OUT_6_Pin GPIO_PIN_1
#define PWM_OUT_6_GPIO_Port GPIOA
#define Relay_1_Pin GPIO_PIN_2
#define Relay_1_GPIO_Port GPIOA
#define Relay_2_Pin GPIO_PIN_3
#define Relay_2_GPIO_Port GPIOA
#define TIM1_CH1_Pin GPIO_PIN_9
#define TIM1_CH1_GPIO_Port GPIOE
#define TIM1_CH2_Pin GPIO_PIN_11
#define TIM1_CH2_GPIO_Port GPIOE
#define TIM1_CH3_Pin GPIO_PIN_13
#define TIM1_CH3_GPIO_Port GPIOE
#define TIM1_CH4_Pin GPIO_PIN_14
#define TIM1_CH4_GPIO_Port GPIOE
#define TIM12_CH1_Pin GPIO_PIN_14
#define TIM12_CH1_GPIO_Port GPIOB
#define TIM12_CH2_Pin GPIO_PIN_15
#define TIM12_CH2_GPIO_Port GPIOB
#define PWM_OUT_1_Pin GPIO_PIN_12
#define PWM_OUT_1_GPIO_Port GPIOD
#define PWM_OUT_2_Pin GPIO_PIN_13
#define PWM_OUT_2_GPIO_Port GPIOD
#define PWM_OUT_3_Pin GPIO_PIN_14
#define PWM_OUT_3_GPIO_Port GPIOD
#define PWM_OUT_4_Pin GPIO_PIN_15
#define PWM_OUT_4_GPIO_Port GPIOD
#define RS485_2_Pin GPIO_PIN_3
#define RS485_2_GPIO_Port GPIOD
#define RS485_1_Pin GPIO_PIN_4
#define RS485_1_GPIO_Port GPIOD
#define Led_Core_Pin GPIO_PIN_7
#define Led_Core_GPIO_Port GPIOB
#define Led_1_Pin GPIO_PIN_0
#define Led_1_GPIO_Port GPIOE
#define Led_2_Pin GPIO_PIN_1
#define Led_2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
