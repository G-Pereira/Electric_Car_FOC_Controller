/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define encoder_w1_Pin GPIO_PIN_0
#define encoder_w1_GPIO_Port GPIOC
#define encoder_w2_Pin GPIO_PIN_1
#define encoder_w2_GPIO_Port GPIOC
#define braking_pedal_Pin GPIO_PIN_2
#define braking_pedal_GPIO_Port GPIOC
#define accelerator_pedal_Pin GPIO_PIN_3
#define accelerator_pedal_GPIO_Port GPIOC
#define Current_PH1_Pin GPIO_PIN_0
#define Current_PH1_GPIO_Port GPIOA
#define Current_PH2_Pin GPIO_PIN_1
#define Current_PH2_GPIO_Port GPIOA
#define Current_PH3_Pin GPIO_PIN_2
#define Current_PH3_GPIO_Port GPIOA
#define DC_bus_voltage_Pin GPIO_PIN_3
#define DC_bus_voltage_GPIO_Port GPIOA
#define Accel_CS_Pin GPIO_PIN_4
#define Accel_CS_GPIO_Port GPIOA
#define FOC_IC_CS_Pin GPIO_PIN_5
#define FOC_IC_CS_GPIO_Port GPIOA
#define DC_bus_current_Pin GPIO_PIN_6
#define DC_bus_current_GPIO_Port GPIOA
#define Motor_Voltage_Pin GPIO_PIN_7
#define Motor_Voltage_GPIO_Port GPIOA
#define Gyro_CS_Pin GPIO_PIN_4
#define Gyro_CS_GPIO_Port GPIOC
#define Magnet_CS_Pin GPIO_PIN_5
#define Magnet_CS_GPIO_Port GPIOC
#define Motor_temp_Pin GPIO_PIN_0
#define Motor_temp_GPIO_Port GPIOB
#define Inverter_temp_Pin GPIO_PIN_1
#define Inverter_temp_GPIO_Port GPIOB
#define FOC_Status_Pin GPIO_PIN_6
#define FOC_Status_GPIO_Port GPIOC
#define CAN_STBY_Pin GPIO_PIN_10
#define CAN_STBY_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
