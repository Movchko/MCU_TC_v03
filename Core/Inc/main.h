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
#define MCU_TC_NUM_ADC_CHANNEL 2
#define MCU_TC_FILTERSIZE      128

extern ADC_HandleTypeDef hadc1;
extern uint16_t MCU_TC_ADC_VAL[MCU_TC_NUM_ADC_CHANNEL];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_LINE1_L_Pin GPIO_PIN_0
#define ADC_LINE1_L_GPIO_Port GPIOC
#define ADC_LINE1_H_Pin GPIO_PIN_3
#define ADC_LINE1_H_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define LINE1_EN_Pin GPIO_PIN_2
#define LINE1_EN_GPIO_Port GPIOA
#define SWITCH_Pin GPIO_PIN_2
#define SWITCH_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_6
#define CS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
