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
#define CONT_TEST_EN_Pin GPIO_PIN_13
#define CONT_TEST_EN_GPIO_Port GPIOC
#define BATT_VOLT_READ_Pin GPIO_PIN_0
#define BATT_VOLT_READ_GPIO_Port GPIOC
#define LED_INDICATOR_Pin GPIO_PIN_1
#define LED_INDICATOR_GPIO_Port GPIOC
#define MAIN_L_Pin GPIO_PIN_0
#define MAIN_L_GPIO_Port GPIOA
#define MAIN_H_Pin GPIO_PIN_1
#define MAIN_H_GPIO_Port GPIOA
#define ACCEL_INT_Pin GPIO_PIN_4
#define ACCEL_INT_GPIO_Port GPIOC
#define GYRO_INT_Pin GPIO_PIN_5
#define GYRO_INT_GPIO_Port GPIOC
#define DROGUE_CONT_Pin GPIO_PIN_0
#define DROGUE_CONT_GPIO_Port GPIOB
#define MAIN_CONT_Pin GPIO_PIN_1
#define MAIN_CONT_GPIO_Port GPIOB
#define MAG_CS_Pin GPIO_PIN_12
#define MAG_CS_GPIO_Port GPIOB
#define MAG_INT_Pin GPIO_PIN_6
#define MAG_INT_GPIO_Port GPIOC
#define SD_DET_Pin GPIO_PIN_7
#define SD_DET_GPIO_Port GPIOC
#define DROGUE_H_Pin GPIO_PIN_8
#define DROGUE_H_GPIO_Port GPIOA
#define IO_RF_Pin GPIO_PIN_11
#define IO_RF_GPIO_Port GPIOA
#define GYRO_CS_Pin GPIO_PIN_12
#define GYRO_CS_GPIO_Port GPIOA
#define DROGUE_L_Pin GPIO_PIN_6
#define DROGUE_L_GPIO_Port GPIOB
#define BARO_CS_Pin GPIO_PIN_8
#define BARO_CS_GPIO_Port GPIOB
#define ACCEL_CS_Pin GPIO_PIN_9
#define ACCEL_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
