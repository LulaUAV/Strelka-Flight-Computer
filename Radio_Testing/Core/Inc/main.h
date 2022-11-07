/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define RF_SPI_NSS_Pin GPIO_PIN_4
#define RF_SPI_NSS_GPIO_Port GPIOA
#define RESET_RF_Pin GPIO_PIN_0
#define RESET_RF_GPIO_Port GPIOB
#define IO0_RF_Pin GPIO_PIN_1
#define IO0_RF_GPIO_Port GPIOB
#define IO0_RF_EXTI_IRQn EXTI1_IRQn
#define IO1_RF_Pin GPIO_PIN_10
#define IO1_RF_GPIO_Port GPIOB
#define IO1_RF_EXTI_IRQn EXTI15_10_IRQn
#define IO2_RF_Pin GPIO_PIN_11
#define IO2_RF_GPIO_Port GPIOB
#define IO2_RF_EXTI_IRQn EXTI15_10_IRQn
#define IO3_RF_Pin GPIO_PIN_12
#define IO3_RF_GPIO_Port GPIOB
#define IO3_RF_EXTI_IRQn EXTI15_10_IRQn
#define IO4_RF_Pin GPIO_PIN_13
#define IO4_RF_GPIO_Port GPIOB
#define IO4_RF_EXTI_IRQn EXTI15_10_IRQn
#define IO5_RF_Pin GPIO_PIN_8
#define IO5_RF_GPIO_Port GPIOA
#define INDICATOR_LED_Pin GPIO_PIN_2
#define INDICATOR_LED_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
