/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define encoder_L_A_Pin GPIO_PIN_0
#define encoder_L_A_GPIO_Port GPIOA
#define encoder_L_B_Pin GPIO_PIN_1
#define encoder_L_B_GPIO_Port GPIOA
#define moter_R_INB_Pin GPIO_PIN_5
#define moter_R_INB_GPIO_Port GPIOC
#define moter_R_INA_Pin GPIO_PIN_6
#define moter_R_INA_GPIO_Port GPIOC
#define moter_R_PWM_Pin GPIO_PIN_8
#define moter_R_PWM_GPIO_Port GPIOC
#define moter_L_PWM_Pin GPIO_PIN_9
#define moter_L_PWM_GPIO_Port GPIOC
#define encoder_R_A_Pin GPIO_PIN_8
#define encoder_R_A_GPIO_Port GPIOA
#define encoder_R_B_Pin GPIO_PIN_9
#define encoder_R_B_GPIO_Port GPIOA
#define moter_L_INA_Pin GPIO_PIN_8
#define moter_L_INA_GPIO_Port GPIOB
#define moter_L_INB_Pin GPIO_PIN_9
#define moter_L_INB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
