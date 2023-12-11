/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BR_IN4_Pin GPIO_PIN_0
#define BR_IN4_GPIO_Port GPIOC
#define BR_IN3_Pin GPIO_PIN_1
#define BR_IN3_GPIO_Port GPIOC
#define BL_IN1_Pin GPIO_PIN_2
#define BL_IN1_GPIO_Port GPIOC
#define BL_IN2_Pin GPIO_PIN_3
#define BL_IN2_GPIO_Port GPIOC
#define BRA_TIM2_CH3_Pin GPIO_PIN_10
#define BRA_TIM2_CH3_GPIO_Port GPIOB
#define BLB_TIM2_CH4_Pin GPIO_PIN_11
#define BLB_TIM2_CH4_GPIO_Port GPIOB
#define FRB_TIM1_CH1_Pin GPIO_PIN_8
#define FRB_TIM1_CH1_GPIO_Port GPIOA
#define FLA_TIM1_CH4_Pin GPIO_PIN_11
#define FLA_TIM1_CH4_GPIO_Port GPIOA
#define FR_IN4_Pin GPIO_PIN_3
#define FR_IN4_GPIO_Port GPIOB
#define FR_IN3_Pin GPIO_PIN_4
#define FR_IN3_GPIO_Port GPIOB
#define FL_IN2_Pin GPIO_PIN_8
#define FL_IN2_GPIO_Port GPIOB
#define FL_IN1_Pin GPIO_PIN_9
#define FL_IN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
