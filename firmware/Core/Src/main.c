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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "sd_wr.h"
#include "IMU_read.h"
#include "adcUnitConversion.h"
#include "encoderMode.h"
#include "FOC_lib.h"
#include "time.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define	NR_ADC_CHANNELS 12 //Nº de channels adc


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


	//SD file variables
	FATFS fs;  // file system
	FIL fil[15];  // file
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
	float dc_current, current_ph1, current_ph2, current_ph3, dc_voltage, voltage_ph1, voltage_ph2, voltage_ph3,  motor_temp, conv_temp, acc_pedal , brk_pedal;

	//String aux
	char str[30], stamp[5];

	//Encoder mode variables
	uint32_t counter1 = 0, counter2 = 0;
	uint32_t tick = 0;
	float speed = 0;

	//RTC
	RTC_TimeTypeDef currentTime;
	RTC_DateTypeDef currentDate;
	uint32_t time_subsec=0;
	uint16_t msec_stamp=0;


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

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	time_subsec=HAL_GetTick();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  HAL_GPIO_WritePin(Accel_CS_GPIO_Port, Accel_CS_Pin, SET);
  HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, SET);
  HAL_GPIO_WritePin(Magnet_CS_GPIO_Port, Magnet_CS_Pin, SET);
  HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //Initialize FOC-IC registers
  //FP=0,707 ??
  foc_ic_config(&hspi2);



  HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, RESET);
  //HAL_SPI_Transmit(&hspi2, /*reg*/ , /*size*/ , 2000);
  HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);
  //Initialize IMU



  //IMU_config(&hspi2); é para ativar dps

  //Initialize data logger
  if(mount_card (&fs) != FR_OK){
	  printf("ERROR mounting SD Card");
  }


  HAL_ADC_Start_DMA(&hadc1, buffer_dma, NR_ADC_CHANNELS);

  //Initialize encoder mode
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  tick = HAL_GetTick();
  counter1 = __HAL_TIM_GET_COUNTER(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //convert the 3 adc current values from the motor
	  //CURRENT PH1
	  current_ph1 = motorCurrent(adc_dma[0]);
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", current_ph1);
	  update_file("phase1_current.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[7]), &bw);

	  //CURRENT PH2
	  current_ph2 = motorCurrent(adc_dma[1]);
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", current_ph2);
	  update_file("phase2_current.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[8]), &bw);

	  //CURRENT PH3
	  current_ph3= motorCurrent(adc_dma[2]);
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", current_ph3);
	  update_file("phase3_current.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[9]), &bw);


	  //convert the 3 adc voltage values from the motor
	  voltage_ph1 = voltageAC(adc_dma[3]);
	  voltage_ph2 = voltageAC(adc_dma[4]);
	  voltage_ph3 = voltageAC(adc_dma[5]);


	  //MOTOR TEMP
	  motor_temp = motorTemp(adc_dma[6]); //convert adc value for temperature on motor
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", motor_temp);
	  update_file("Motor_temperature.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[10]), &bw);


	  //CONV TEMP
	  conv_temp = igbtTemp(adc_dma[7]); //convert adc value for temperature on converter
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", conv_temp);
	  update_file("Converter_temperature.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[11]), &bw);


	  //DC CURRENT
	  dc_current = motorCurrent(adc_dma[8]); //convert adc value for current on DC bus
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", dc_current);
	  update_file("DC_current.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[6]), &bw);


	  dc_voltage = voltageDC(adc_dma[9]); //convert adc value for voltage on DC bus


	  //BRK pedal
	  brk_pedal = pedalPos(adc_dma[10]); //convert adc value for braking pedal
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", brk_pedal);
	  update_file("Braking_Pedal.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[14]), &bw);


	  //THROTTLE
	  acc_pedal = pedalPos(adc_dma[11]); //convert adc value for throttle
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", acc_pedal);
	  update_file("Accelerator_pedal.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[13]), &bw);


	  //ENCODER
	  //read and convert adc values for encoder
	  ////counter1 = __HAL_TIM_GET_COUNTER(&htim2);
	  ///HAL_Delay(500);
	  if(HAL_GetTick() - tick > 1000L){
		  counter2 = __HAL_TIM_GET_COUNTER(&htim2);
		  speed = motorSpeed(&counter1, counter2, &tick, htim2);
	  }

	  msec_stamp=HAL_GetTick()-time_subsec; //aw shit
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%f ", speed);
	  update_file("encoder_data.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[12]), &bw);


	  //ACCELEROMETER
	  IMU_acc_read(&hspi2, accel_data); //read and convert accelerometer data
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%d ", accel_data[0]);

	  //Send read data to SD card
	  update_file("accelerometer_data_x.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[0]), &bw);
	  sprintf(str, "%d ", accel_data[1]);
	  update_file("accelerometer_data_y.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[1]), &bw);
	  sprintf(str, "%d ", accel_data[2]);
	  update_file("accelerometer_data_z.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[2]), &bw);

	  //GYROSCOPE
	  IMU_gyro_read(&hspi2, gyro_data); //read and convert gyroscope data
	  msec_stamp=HAL_GetTick()-time_subsec;
	  sprintf(stamp, "%d", msec_stamp);
	  sprintf(str, "%d ", gyro_data[0]);

	  //Send read data to SD card
	  update_file("gyroscope_data_x.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[3]), &bw);
	  sprintf(str, "%d ", gyro_data[1]);
	  update_file("gyroscope_data_y.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[4]), &bw);
	  sprintf(str, "%d ", gyro_data[2]);
	  update_file("gyroscope_data_z.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[5]), &bw);

	  //DEFINIR O QUE FAZER COM VALORES LIDOS

	  //send torque reference to foc ic

	  //read and convert adc value for accelerator potenciometer

	  //after reading pedal value send ref to FOC-IC

	  //save read values on SD card separate files


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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_2);
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
