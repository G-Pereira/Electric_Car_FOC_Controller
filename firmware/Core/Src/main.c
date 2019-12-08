/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "sd_wr.h"
#include "IMU_read.h"
#include "adcUnitConversion.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define	NR_ADC_CHANNELS 5 //Nº de channels adc



	//

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


	//SD file variables
	FATFS fs;  // file system
	FIL fil;  // file
	FRESULT fresult;  // to store the result
	char buffer[512]; // to store data

	UINT br, bw;   // file read/write count

	/* capacity related variables */
	//DWORD fre_clust;
	DWORD total, free_space;

	//IMU accelerometer data
	int accel_data[3];
	//IMU accelerometer data
	int gyro_data[3];

	//ADCs converted values variables
	int dc_current, current_ph1, current_ph2, current_ph3,  motor_temp, conv_temp, enc_data, acc_pedal , brk_pedal;

	//String aux
	char *str;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint32_t adc_dma[NR_ADC_CHANNELS], buffer_dma[NR_ADC_CHANNELS];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(int i=0; i < NR_ADC_CHANNELS; i++)
	{
		adc_dma[i]=buffer_dma[i];
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float curr_ph1 = 0;
	float curr_ph2 = 0;
	float curr_ph3 = 0;
	float temp_motor = 0;
	float temp_inv = 0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  /* USER CODE BEGIN 2 */

  //Initialize FOC-IC registers
  //FP=0,707
  foc_ic_config(&hspi2);



  HAL_GPIO_WritePin(FOC_IC_CSS_GPIO_Port, FOC_IC_CSS_Pin, RESET);
  //HAL_SPI_Transmit(&hspi2, /*reg*/ , /*size*/ , 2000);
  HAL_GPIO_WritePin(FOC_IC_CSS_GPIO_Port, FOC_IC_CSS_Pin, SET);
  //Initialize IMU



  IMU_config(&hspi2);

  //Initialize data logger
  if(mount_card (&fs) != FR_OK){
	  printf("ERROR mounting SD Card");
  }


  HAL_ADC_Start_DMA(&hadc1, buffer_dma, NR_ADC_CHANNELS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //read and convert adc value for current on DC bus

	  //convert adc value for voltage on DC bus

	  //convert the 3 adc current values from the motor

	  //convert the 3 adc current values from the motor
	  curr_ph1 = motorCurrent(adc_dma[0]);
	  curr_ph2 = motorCurrent(adc_dma[1]);
	  curr_ph3 = motorCurrent(adc_dma[2]);


	  temp_motor = motorCurrent(adc_dma[3]); //convert adc value for temperature on motor
	  temp_inv = motorCurrent(adc_dma[4]); //convert adc value for temperature on converter

	  //read and convert adc values for encoder

	  //read and convert adc value for braking pedal


	  IMU_acc_read(&hspi2, accel_data); //read and convert accelerometer data
	  sprintf(str, "%d", accel_data[0]);

	  //Send read data to SD card
	  update_file("accelerometer_data_x.txt", str, &fil, &bw);
	  sprintf(str, "%d", accel_data[1]);
	  update_file("accelerometer_data_y.txt", str, &fil, &bw);
	  sprintf(str, "%d", accel_data[2]);
	  update_file("accelerometer_data_z.txt", str, &fil, &bw);

	  //read and convert gyroscope data
	  IMU_gyro_read(&hspi2, gyro_data);
	  sprintf(str, "%d", gyro_data[0]);

	  //Send read data to SD card
	  update_file("gyroscope_data_x.txt", str, &fil, &bw);
	  sprintf(str, "%d", gyro_data[1]);
	  update_file("gyroscope_data_y.txt", str, &fil, &bw);
	  sprintf(str, "%d", gyro_data[2]);
	  update_file("gyroscope_data_z.txt", str, &fil, &bw);

	  //DEFINIR O QUE FAZER COM VALORES LIDOS

	  //send torque reference to foc ic

	  //read and convert adc value for accelerator potenciometer

	  //after reading pedal value send ref to FOC-IC

	  //save read values on SD card separate files

	  sprintf(str, "%d", dc_current);
	  update_file("DC_current.txt", str, &fil, &bw);
	  sprintf(str, "%d", current_ph1);
	  update_file("phase1_current.txt", str, &fil, &bw);
	  sprintf(str, "%d", current_ph2);
	  update_file("phase2_current.txt", str, &fil, &bw);
	  sprintf(str, "%d", current_ph3);
	  update_file("phase3_current.txt", str, &fil, &bw);
	  sprintf(str, "%d", motor_temp);
	  update_file("Motor_temperature.txt", str, &fil, &bw);
	  sprintf(str, "%d", conv_temp);
	  update_file("Converter_temperature.txt", str, &fil, &bw);
	  sprintf(str, "%d", enc_data);
	  update_file("encoder_data.txt", str, &fil, &bw);
	  sprintf(str, "%d", acc_pedal);
	  update_file("Accelerator_pedal.txt", str, &fil, &bw);
	  sprintf(str, "%d", brk_pedal);
	  update_file("Braking_Pedal.txt", str, &fil, &bw);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
