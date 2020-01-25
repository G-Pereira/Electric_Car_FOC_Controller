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
#define curr_u_Pin GPIO_PIN_0
#define curr_u_GPIO_Port GPIOC
#define curr_v_Pin GPIO_PIN_1
#define curr_v_GPIO_Port GPIOC
#define brk_pedal_Pin GPIO_PIN_2
#define brk_pedal_GPIO_Port GPIOC
#define acc_pedal_Pin GPIO_PIN_3
#define acc_pedal_GPIO_Port GPIOC
#define enc_a_Pin GPIO_PIN_0
#define enc_a_GPIO_Port GPIOA
#define enc_b_Pin GPIO_PIN_1
#define enc_b_GPIO_Port GPIOA
#define curr_w_Pin GPIO_PIN_2
#define curr_w_GPIO_Port GPIOA
#define Accel_CS_Pin GPIO_PIN_4
#define Accel_CS_GPIO_Port GPIOA
#define Gyro_CS_Pin GPIO_PIN_2
#define Gyro_CS_GPIO_Port GPIOB
#define Magnet_CS_Pin GPIO_PIN_10
#define Magnet_CS_GPIO_Port GPIOB
#define SPI_CS_FOC_Pin GPIO_PIN_12
#define SPI_CS_FOC_GPIO_Port GPIOB
#define SD_Det_Pin GPIO_PIN_7
#define SD_Det_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
