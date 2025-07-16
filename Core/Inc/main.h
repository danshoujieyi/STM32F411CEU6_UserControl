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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IIC_SCL3_Pin GPIO_PIN_1
#define IIC_SCL3_GPIO_Port GPIOB
#define IIC_SDA3_Pin GPIO_PIN_2
#define IIC_SDA3_GPIO_Port GPIOB
#define IIC_SCL4_Pin GPIO_PIN_12
#define IIC_SCL4_GPIO_Port GPIOB
#define IIC_SDA4_Pin GPIO_PIN_13
#define IIC_SDA4_GPIO_Port GPIOB
#define IIC_SCL1_Pin GPIO_PIN_14
#define IIC_SCL1_GPIO_Port GPIOB
#define IIC_SDA1_Pin GPIO_PIN_15
#define IIC_SDA1_GPIO_Port GPIOB
#define IIC_SCL2_Pin GPIO_PIN_8
#define IIC_SCL2_GPIO_Port GPIOB
#define IIC_SDA2_Pin GPIO_PIN_9
#define IIC_SDA2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
