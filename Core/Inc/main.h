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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEMP_MOTOR_Pin GPIO_PIN_0
#define TEMP_MOTOR_GPIO_Port GPIOC
#define VIN_SENS_Pin GPIO_PIN_2
#define VIN_SENS_GPIO_Port GPIOC
#define SENS3_Pin GPIO_PIN_0
#define SENS3_GPIO_Port GPIOA
#define SENS2_Pin GPIO_PIN_1
#define SENS2_GPIO_Port GPIOA
#define SENS1_Pin GPIO_PIN_2
#define SENS1_GPIO_Port GPIOA
#define TEMP_MOSFET_Pin GPIO_PIN_3
#define TEMP_MOSFET_GPIO_Port GPIOA
#define ADC_EXT_Pin GPIO_PIN_5
#define ADC_EXT_GPIO_Port GPIOA
#define ADC_EXT2_Pin GPIO_PIN_6
#define ADC_EXT2_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_4
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOC
#define CURR2_Pin GPIO_PIN_0
#define CURR2_GPIO_Port GPIOB
#define CURR1_Pin GPIO_PIN_1
#define CURR1_GPIO_Port GPIOB
#define DC_CAL_Pin GPIO_PIN_12
#define DC_CAL_GPIO_Port GPIOB
#define EN_GATE_Pin GPIO_PIN_10
#define EN_GATE_GPIO_Port GPIOC
#define HALL3_Pin GPIO_PIN_11
#define HALL3_GPIO_Port GPIOC
#define HALL3_EXTI_IRQn EXTI15_10_IRQn
#define FAULT_Pin GPIO_PIN_12
#define FAULT_GPIO_Port GPIOC
#define HALL1_Pin GPIO_PIN_6
#define HALL1_GPIO_Port GPIOB
#define HALL1_EXTI_IRQn EXTI9_5_IRQn
#define HALL2_Pin GPIO_PIN_7
#define HALL2_GPIO_Port GPIOB
#define HALL2_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
