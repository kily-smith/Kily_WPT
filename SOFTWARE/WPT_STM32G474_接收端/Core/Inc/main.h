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
#include "stm32g4xx_hal.h"

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
#define RGB_B_Pin GPIO_PIN_0
#define RGB_B_GPIO_Port GPIOA
#define RGB_G_Pin GPIO_PIN_1
#define RGB_G_GPIO_Port GPIOA
#define RGB_R_Pin GPIO_PIN_2
#define RGB_R_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_1
#define BUZZ_GPIO_Port GPIOB
#define ADC_V_NTC1_Pin GPIO_PIN_12
#define ADC_V_NTC1_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_13
#define SPI1_CS_GPIO_Port GPIOB
#define SPI1_RST_Pin GPIO_PIN_14
#define SPI1_RST_GPIO_Port GPIOB
#define SPI1_DC_Pin GPIO_PIN_15
#define SPI1_DC_GPIO_Port GPIOB
#define SPI1_BL_Pin GPIO_PIN_6
#define SPI1_BL_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
