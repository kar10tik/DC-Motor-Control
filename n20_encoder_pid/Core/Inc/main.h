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
#define PWM_PID_Pin GPIO_PIN_0
#define PWM_PID_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOA
#define DIR_1_Pin GPIO_PIN_6
#define DIR_1_GPIO_Port GPIOB
#define DIR_2_Pin GPIO_PIN_7
#define DIR_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PPR 7 //Pulses per revolution for motor encoder
#define KP 10
#define KI 30
#define KD 20
#define PID_SCALE 100 // Scale factor for Kp, Ki, Kd
#define MAX_PWM 255  // Maximum PWM value
#define MIN_PWM 0    // Minimum PWM value
#define INTEGRAL_MAX 256 // Saturation for integral term
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
