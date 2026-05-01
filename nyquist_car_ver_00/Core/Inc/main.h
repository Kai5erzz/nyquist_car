/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define MOTOR_ISEN2_Pin GPIO_PIN_0
#define MOTOR_ISEN2_GPIO_Port GPIOC
#define MOTOR_ISEN3_Pin GPIO_PIN_1
#define MOTOR_ISEN3_GPIO_Port GPIOC
#define MOTOR_ISEN1_Pin GPIO_PIN_4
#define MOTOR_ISEN1_GPIO_Port GPIOA
#define MOTOR_ISEN4_Pin GPIO_PIN_4
#define MOTOR_ISEN4_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_7
#define LED0_GPIO_Port GPIOH
#define AD2_Pin GPIO_PIN_8
#define AD2_GPIO_Port GPIOH
#define AD0_Pin GPIO_PIN_6
#define AD0_GPIO_Port GPIOC
#define AD1_Pin GPIO_PIN_8
#define AD1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
