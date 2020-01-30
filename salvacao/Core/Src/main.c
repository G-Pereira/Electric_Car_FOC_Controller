/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "sd_wr.h"
#include "IMU_read.h"
#include "encoderMode.h"
#include "adcUnitConversion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	NR_ADC_CHANNELS 7 //Nº de channels adc
#define TICK_RATE 1 //milisecond

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

volatile unsigned long SystemTick=0;
volatile unsigned long __unix_ms=0;
volatile unsigned long __unix_sec=0;

uint32_t adc_dma[NR_ADC_CHANNELS], buffer_dma[NR_ADC_CHANNELS];

//SD file variables
	FATFS fs;  // file system
	FIL fil, fil2, fil3;  // file
	FRESULT fresult;  // to store the result
	char buffer[512]; // to store data

	UINT br, bw;   // file read/write count

	/* capacity related variables */
	//DWORD fre_clust;
	DWORD total, free_space;

//Encoder mode variables
	uint32_t counter1 = 0, counter2 = 0;
	uint32_t tick = 0;
	float speed = 0;

//IMU accelerometer data
	int accel_data[3];
//IMU accelerometer data
	int gyro_data[3];
	long int c1;

	float dc_current, current_ph1, current_ph2, current_ph3, dc_voltage, voltage_ph1, voltage_ph2, voltage_ph3,  motor_temp, conv_temp, acc_pedal , brk_pedal;

//RTC
/*	RTC_TimeTypeDef currentTime;
	RTC_DateTypeDef currentDate;
	uint32_t time_subsec = 0;
	uint32_t time_stamp = 0;
	char stamp[50];
*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1){
		for(int i=0; i < NR_ADC_CHANNELS; i++)
			{
				adc_dma[i]=buffer_dma[i];
			}

		//printf("buff %lu\n", adc_dma[0]);
	}
}

/*void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){

	printf("Interrupcao\n");
	time_subsec = HAL_GetTick();

}
*/
/*void SysTick_Handler(void){
	static uint16_t tick = 0;
	static uint16_t second = 0;
	static uint16_t minute = 0;
	static uint16_t hour = 0;
	printf("tick %d", tick);
	switch (tick++) {
		case 999:
			tick = 0;
			if(second==59){
				second=0;
				if(minute==59){
					minute=0;
					hour++;
				}
				else minute++;
			}
			else second++;
	}

}*/

//void SysTick_Handler(void){

	//HAL_IncTick();
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//UNUSED(htim); será preciso?

	//counter2 = __HAL_TIM_GET_COUNTER(&htim2);
	//printf("%lu\n", HAL_GetTick());
	speed = motorSpeed(&counter1, &tick, htim2);

	/* passa a fazer-se aqui?
	sprintf(str, "%f ", speed);
	update_file("encoder_data.txt", str, get_timestamp(&hrtc, &currentTime, &currentDate), stamp, &(fil[12]), &bw); */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  printf("hallo");


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(Accel_CS_GPIO_Port, Accel_CS_Pin, SET);
  HAL_GPIO_WritePin(Gyro_CS_GPIO_Port, Gyro_CS_Pin, SET);
  HAL_GPIO_WritePin(Magnet_CS_GPIO_Port, Magnet_CS_Pin, SET);
  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);

  fresult = f_mount(&fs, "", 0 );
  if(fresult == FR_OK){
	  printf("Mount feito\n");
  }
  fresult = f_open(&fil, "agora17.txt", FA_CREATE_ALWAYS | FA_WRITE);
  if(fresult != FR_OK){
  	  printf("agora17.txt fodeu\n");
  }
  fresult = f_printf(&fil, "kay\n");
  if(fresult != FR_OK){
	  printf("agora17.txt fprintf falhou\n");
  }
  f_close(&fil);


  fresult = update_file("test.txt", "hey\n", "", "", &fil, &bw);
  if(fresult!=FR_OK){
	  printf("test.txt fodeu\n");
  }

  char str2[30];

  IMU_config(&hspi2);

  HAL_TIM_Base_Start_IT(&htim6);

  //Initialize encoder mode
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  tick = HAL_GetTick();
  counter1 = __HAL_TIM_GET_COUNTER(&htim2);

  uint8_t aux[5], aux2[1], aux3[1], str3[4], str[5];
  uint8_t ref[5];
    aux[0]=0b10000001;
    for(int i=1; i<=4; i++){
  	  aux[i]=0b00000000;
    }
    aux2[0]=0b00000000;


	HAL_ADC_Start_DMA(&hadc1, buffer_dma, NR_ADC_CHANNELS);


	fresult = update_file("test2.txt", "hey", "", "", &fil, &bw);
	if(fresult!=FR_OK){
		printf("test.txt fodeu\n");
	}

    for(int i=0; i<4; i++){
    	  printf("aux %d - %d\n", i, aux[i]);
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //printf("time_subsec %lu\n", time_subsec);

	  uint32_t read=adc_dma[0];
	  motor_temp = motorTemp(read);
	  printf("temp %f\n", motor_temp);
	  read=adc_dma[1];
	  float brk = pedalPos(read);
	  printf("brk %f", brk);
	  read=adc_dma[2];
	  float acc = pedalPos(read);
	  printf("acc %f", acc);

	  read=adc_dma[3];
	  float c1_f = motorCurrent(read);
	  c1=c1_f;
	  printf("current1 %l", c1);
	  read=adc_dma[4];
	  float c2 = motorCurrent(read);
	  read=adc_dma[5];
	  float c3 = motorCurrent(read);

	  read=adc_dma[6];
	  conv_temp = igbtTemp(read);
	  printf("temp %f\n", conv_temp);
/*
	  if(acc>50){
		  ref[0]
		  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
		  HAL_SPI_Transmit(&hspi2, ref, 5, 1000);
		  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);
	  }
	  else if(brk>50)*/


	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
	  HAL_SPI_Transmit(&hspi2, aux, 5, 1000);
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);
	  HAL_Delay(1);


	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
	  HAL_SPI_Transmit(&hspi2, aux2, 1, 1000);
	  HAL_Delay(1);
	  HAL_SPI_Receive(&hspi2, str, 4, 1000);
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);
	  //HAL_SPI_Receive(&hspi2, str, 4, 200);
	  printf("Aquiii\n");
	  printf("%d %d %d %d\n", str[0], str[1], str[2], str[3]);


	  //ler velocidades do tmc
	  aux3[0]=0x6A;
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
	  HAL_SPI_Transmit(&hspi2, aux3, 1, 1000);
	  HAL_Delay(1);
	  HAL_SPI_Receive(&hspi2, str3, 4, 1000);
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);
	  printf("PID VELOCITY ACTUAL: %d %d %d %d\n", str3[0], str3[1], str3[2], str3[3]);

	  aux3[0]=0x22;
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
	  HAL_SPI_Transmit(&hspi2, aux3, 1, 1000);
	  HAL_Delay(1);
	  HAL_SPI_Receive(&hspi2, str3, 4, 1000);
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);
	  printf("OPENLOOP VELOCITY ACTUAL: %d %d %d %d\n", str3[0], str3[1], str3[2], str3[3]);

	  aux3[0]=0x41;
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
	  HAL_SPI_Transmit(&hspi2, aux3, 1, 1000);
	  HAL_Delay(1);
	  HAL_SPI_Receive(&hspi2, str3, 4, 1000);
	  HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);
	  printf("AENC DECODER COUNT: %d %d %d %d\n", str3[0], str3[1], str3[2], str3[3]);

	  sprintf(str2,"%f ", acc);
	  //time_stamp = HAL_GetTick()-time_subsec;
	  //sprintf(stamp, "%s", time_stamp);
	  //printf("HAL_GetTick() - %lu  time-subsec - %lu \n", HAL_GetTick(), time_subsec);
	  char stamp[100];
	  sprintf(stamp, "%d:%d",  __unix_sec,  __unix_ms);
	  FRESULT res = update_file("meio.txt", str2, stamp, "", &fil2, &bw);


	  //update_file("teste.txt", "chico da tina", "ah", "ah\n", &fil, &bw);
	  IMU_acc_read(&hspi2, accel_data);
	  for(int i=0; i<=2; i++){
		  sprintf(str2, "%d ", accel_data[i]);
		  /*res = update_file("acc.txt", str2, get_timestamp(&hrtc, &currentTime, &currentDate), "", &fil2, &bw);
		  printf("timestamp - %s\n", get_timestamp(&hrtc, &currentTime, &currentDate));
		  if(res!=FR_OK){
			  printf("ainda nao\n");
		  }
		  //printf("acc[%d]= %s  ", i, str2);*/
	  }


	  IMU_gyro_read(&hspi2, gyro_data);
	  for(int i=0; i<=2; i++){
		  sprintf(str2, "%d ", gyro_data[i]);
		  //printf("gyro[%d]= %s  ", i, str2);
	  }

	  printf("TIME - %d", __unix_sec);

	  //printf("speed: %f\n", speed);
	  //printf("counter1 = %lu\n", __HAL_TIM_GET_COUNTER(&htim2));

	 //ENCODER
	 //read and convert adc values for encoder
	 		  ////counter1 = __HAL_TIM_GET_COUNTER(&htim2);
	 		  ///HAL_Delay(500);
	 		  /*if(HAL_GetTick() - tick > 1000L){
	 			  counter2 = __HAL_TIM_GET_COUNTER(&htim2);
	 			  speed = motorSpeed(&counter1, counter2, &tick, htim2);
	 		  }*/


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_2);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 255;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 48000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Accel_CS_GPIO_Port, Accel_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Gyro_CS_Pin|Magnet_CS_Pin|SPI_CS_FOC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Accel_CS_Pin */
  GPIO_InitStruct.Pin = Accel_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Accel_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Gyro_CS_Pin Magnet_CS_Pin SPI_CS_FOC_Pin */
  GPIO_InitStruct.Pin = Gyro_CS_Pin|Magnet_CS_Pin|SPI_CS_FOC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_Det_Pin */
  GPIO_InitStruct.Pin = SD_Det_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Det_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


int __io_putchar(int ch){
	ITM_SendChar(ch);
	return ch;
}

int _write(int file, char *ptr, int len){
	int DataIdx;

	for(DataIdx = 0; DataIdx < len; DataIdx++){
		__io_putchar(*ptr++);
	}
	return len;
}
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