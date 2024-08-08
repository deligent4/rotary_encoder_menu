/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLU_Pin GPIO_PIN_15
#define LED_BLU_GPIO_Port GPIOC
#define A_SW_Pin GPIO_PIN_0
#define A_SW_GPIO_Port GPIOA
#define A_SW_EXTI_IRQn EXTI0_IRQn
#define B_SW_Pin GPIO_PIN_1
#define B_SW_GPIO_Port GPIOA
#define B_SW_EXTI_IRQn EXTI1_IRQn
#define C_SW_Pin GPIO_PIN_4
#define C_SW_GPIO_Port GPIOA
#define C_SW_EXTI_IRQn EXTI4_IRQn
#define ROT_SW_Pin GPIO_PIN_5
#define ROT_SW_GPIO_Port GPIOA
#define ROT_SW_EXTI_IRQn EXTI9_5_IRQn
#define ROT_A_Pin GPIO_PIN_6
#define ROT_A_GPIO_Port GPIOA
#define ROT_B_Pin GPIO_PIN_7
#define ROT_B_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
