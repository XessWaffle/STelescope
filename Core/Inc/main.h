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
#define TRUE 1
#define FALSE 0
#define NULL ((void *)0)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_RTC_Init(void);

/* USER CODE BEGIN EFP */
void step();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ST_MS3_Pin GPIO_PIN_13
#define ST_MS3_GPIO_Port GPIOC
#define ST_MS2_Pin GPIO_PIN_14
#define ST_MS2_GPIO_Port GPIOC
#define ST_MS1_Pin GPIO_PIN_15
#define ST_MS1_GPIO_Port GPIOC
#define ST_DIR_R_Pin GPIO_PIN_1
#define ST_DIR_R_GPIO_Port GPIOA
#define ST_STEP_R_Pin GPIO_PIN_2
#define ST_STEP_R_GPIO_Port GPIOA
#define ST_DIR_Y_Pin GPIO_PIN_3
#define ST_DIR_Y_GPIO_Port GPIOA
#define ST_STEP_Y_Pin GPIO_PIN_4
#define ST_STEP_Y_GPIO_Port GPIOA
#define M6D_INT1_Pin GPIO_PIN_0
#define M6D_INT1_GPIO_Port GPIOB
#define CAM_CS_Pin GPIO_PIN_1
#define CAM_CS_GPIO_Port GPIOB
#define M6D_INT2_Pin GPIO_PIN_2
#define M6D_INT2_GPIO_Port GPIOB
#define DISP_RST_Pin GPIO_PIN_12
#define DISP_RST_GPIO_Port GPIOB
#define DISP_DC_Pin GPIO_PIN_14
#define DISP_DC_GPIO_Port GPIOB
#define DISP_CS1_Pin GPIO_PIN_8
#define DISP_CS1_GPIO_Port GPIOA
#define ST_SLP_Pin GPIO_PIN_15
#define ST_SLP_GPIO_Port GPIOA
#define MDL_DRDY_Pin GPIO_PIN_4
#define MDL_DRDY_GPIO_Port GPIOB
#define MDL_INT_Pin GPIO_PIN_5
#define MDL_INT_GPIO_Port GPIOB
#define ST_DIR_P_Pin GPIO_PIN_8
#define ST_DIR_P_GPIO_Port GPIOB
#define ST_STEP_P_Pin GPIO_PIN_9
#define ST_STEP_P_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
