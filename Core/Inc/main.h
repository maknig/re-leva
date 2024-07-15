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
#include "stm32l4xx_hal.h"

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
#define WATER_LEVEL_EN_2_Pin GPIO_PIN_0
#define WATER_LEVEL_EN_2_GPIO_Port GPIOA
#define WATER_LEVEL_2_Pin GPIO_PIN_1
#define WATER_LEVEL_2_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_2
#define SW1_GPIO_Port GPIOA
#define SW1_EXTI_IRQn EXTI2_IRQn
#define WATER_LEVEL_EN_1_Pin GPIO_PIN_3
#define WATER_LEVEL_EN_1_GPIO_Port GPIOA
#define WATER_LEVEL_1_Pin GPIO_PIN_4
#define WATER_LEVEL_1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOA
#define ADC_T_COFFEE_1_Pin GPIO_PIN_6
#define ADC_T_COFFEE_1_GPIO_Port GPIOA
#define ADC_T_STEAM_Pin GPIO_PIN_7
#define ADC_T_STEAM_GPIO_Port GPIOA
#define ADC_P_COFFE_Pin GPIO_PIN_0
#define ADC_P_COFFE_GPIO_Port GPIOB
#define ADC_T_COFFEE_2_Pin GPIO_PIN_1
#define ADC_T_COFFEE_2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define PUMP_STEAM_Pin GPIO_PIN_15
#define PUMP_STEAM_GPIO_Port GPIOB
#define PUMP_COFFEE_Pin GPIO_PIN_8
#define PUMP_COFFEE_GPIO_Port GPIOA
#define ZERO_CROSS_Pin GPIO_PIN_11
#define ZERO_CROSS_GPIO_Port GPIOA
#define ZERO_CROSS_EXTI_IRQn EXTI15_10_IRQn
#define HEATER_COFFEE_Pin GPIO_PIN_12
#define HEATER_COFFEE_GPIO_Port GPIOA
#define HEATER_STEAM_Pin GPIO_PIN_8
#define HEATER_STEAM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
