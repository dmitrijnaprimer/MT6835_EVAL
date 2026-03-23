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
#include "stm32h5xx_hal.h"

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
#define MT6835_CSN_Pin GPIO_PIN_1
#define MT6835_CSN_GPIO_Port GPIOA
#define LED_0_Pin GPIO_PIN_3
#define LED_0_GPIO_Port GPIOA
#define MT6835_MOSI_Pin GPIO_PIN_4
#define MT6835_MOSI_GPIO_Port GPIOA
#define MT6835_SCK_Pin GPIO_PIN_5
#define MT6835_SCK_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_7
#define LED_2_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_0
#define LED_3_GPIO_Port GPIOB
#define LIR_DA237T_SCK_Pin GPIO_PIN_2
#define LIR_DA237T_SCK_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_10
#define LED_4_GPIO_Port GPIOB
#define MT6835_CAL_EN_Pin GPIO_PIN_12
#define MT6835_CAL_EN_GPIO_Port GPIOB
#define MT6835_PWM_Pin GPIO_PIN_13
#define MT6835_PWM_GPIO_Port GPIOB
#define TMC2225_DIR_Pin GPIO_PIN_9
#define TMC2225_DIR_GPIO_Port GPIOA
#define TMC2225_EN_Pin GPIO_PIN_10
#define TMC2225_EN_GPIO_Port GPIOA
#define LIR_DA237T_DATA_Pin GPIO_PIN_15
#define LIR_DA237T_DATA_GPIO_Port GPIOA
#define TMC2225_STEP_Pin GPIO_PIN_3
#define TMC2225_STEP_GPIO_Port GPIOB
#define MT6835_MISO_Pin GPIO_PIN_4
#define MT6835_MISO_GPIO_Port GPIOB
#define TMC2225_MICROSTEPS_Pin GPIO_PIN_7
#define TMC2225_MICROSTEPS_GPIO_Port GPIOB
#define LED_5_Pin GPIO_PIN_8
#define LED_5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
