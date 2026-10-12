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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mobility_pin_config.h"
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
#define FR_DIR_Pin GPIO_PIN_13
#define FR_DIR_GPIO_Port GPIOC
#define RR_DIR_Pin GPIO_PIN_14
#define RR_DIR_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_12
#define Buzzer_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define LED_YEL_Pin GPIO_PIN_14
#define LED_YEL_GPIO_Port GPIOB
#define LED_GRE_Pin GPIO_PIN_15
#define LED_GRE_GPIO_Port GPIOB
#define FL_DIR_Pin GPIO_PIN_4
#define FL_DIR_GPIO_Port GPIOB
#define RL_DIR_Pin GPIO_PIN_5
#define RL_DIR_GPIO_Port GPIOB
#define FL_Motor_Pin GPIO_PIN_6
#define FL_Motor_GPIO_Port GPIOB
#define RL_Motor_Pin GPIO_PIN_7
#define RL_Motor_GPIO_Port GPIOB
#define FR_Motor_Pin GPIO_PIN_8
#define FR_Motor_GPIO_Port GPIOB
#define RR_Motor_Pin GPIO_PIN_9
#define RR_Motor_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
