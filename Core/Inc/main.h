/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define ir_Pin GPIO_PIN_13
#define ir_GPIO_Port GPIOC
#define ir_EXTI_IRQn EXTI4_15_IRQn
#define Temp1_Pin GPIO_PIN_0
#define Temp1_GPIO_Port GPIOA
#define Temp2_Pin GPIO_PIN_1
#define Temp2_GPIO_Port GPIOA
#define Temp3_Pin GPIO_PIN_2
#define Temp3_GPIO_Port GPIOA
#define Temp4_Pin GPIO_PIN_3
#define Temp4_GPIO_Port GPIOA
#define CT_Pin GPIO_PIN_4
#define CT_GPIO_Port GPIOA
#define Flow_M_2_Pin GPIO_PIN_5
#define Flow_M_2_GPIO_Port GPIOA
#define Flow_M_3_Pin GPIO_PIN_6
#define Flow_M_3_GPIO_Port GPIOA
#define Flow_M_4_Pin GPIO_PIN_7
#define Flow_M_4_GPIO_Port GPIOA
#define Swing_M_4_Pin GPIO_PIN_0
#define Swing_M_4_GPIO_Port GPIOB
#define Swing_M_3_Pin GPIO_PIN_1
#define Swing_M_3_GPIO_Port GPIOB
#define Swing_M_2_Pin GPIO_PIN_2
#define Swing_M_2_GPIO_Port GPIOB
#define Swing_M_1_Pin GPIO_PIN_10
#define Swing_M_1_GPIO_Port GPIOB
#define Heart_Pin GPIO_PIN_11
#define Heart_GPIO_Port GPIOB
#define Flow_M_1_Pin GPIO_PIN_12
#define Flow_M_1_GPIO_Port GPIOB
#define Compressor_Pin GPIO_PIN_13
#define Compressor_GPIO_Port GPIOB
#define Fan_Compressor_Pin GPIO_PIN_14
#define Fan_Compressor_GPIO_Port GPIOB
#define ElecValve_Pin GPIO_PIN_15
#define ElecValve_GPIO_Port GPIOB
#define Fan_Indoor_1_Pin GPIO_PIN_8
#define Fan_Indoor_1_GPIO_Port GPIOA
#define Fan_Indoor_2_Pin GPIO_PIN_9
#define Fan_Indoor_2_GPIO_Port GPIOA
#define Fan_Indoor_3_Pin GPIO_PIN_10
#define Fan_Indoor_3_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
