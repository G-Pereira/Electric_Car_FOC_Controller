//includes
#include "main.h"
#include "IMU_read.h"


void IMU_config(SPI_HandleTypeDef *hspi){

	uint8_t write_words[10];

	//accelerometer config
	write_words[0]=BMX055_RESET_REG;
	write_words[1]=BMX055_INITIATED_SOFT_RESET; //accelerometer soft reset
	write_words[2]=BMX055_ACC_PMU_RANGE_REG;
	write_words[3]=BMX055_ACC_RANGE_2;
	write_words[4]=BMX055_ACC_PMU_BW_REG;
	write_words[5]=BMX055_ACC_PMU_BW_7_81;
	write_words[6]=BMX055_ACC_PMU_LPW_REG;
	write_words[7]=BMX055_ACC_PMU_LPW_MODE_NOMAL|BMX055_ACC_PMU_LPW_SLEEP_DUR_0_5MS;

	HAL_GPIO_WritePin(CS_accel_GPIO_Port, CS_accel_Pin, RESET);
	HAL_SPI_Transmit(hspi, write_words, 8, 2000);
	HAL_Delay(100);
	/*HAL_SPI_Transmit(&hspi1, write_words+2, 2, 2000);
	  	  HAL_Delay(100);
	  	  HAL_SPI_Transmit(&hspi1, write_words+4, 2, 2000);
	  	  HAL_Delay(100);
	  	  HAL_SPI_Transmit(&hspi1, write_words+6, 2, 2000);
	  	  HAL_Delay(100);*/
	HAL_GPIO_WritePin(CS_accel_GPIO_Port, CS_accel_Pin, SET);

	// gyroscope config
	write_words[0]=BMX055_RESET_REG;
	write_words[1]=BMX055_INITIATED_SOFT_RESET;
	write_words[2]=BMX055_GYRO_RANGE_REG;
	write_words[3]=BMX055_GYRO_RANGE_262_4; // Select Gyro Range(262.4 LSB/°/s)
	write_words[4]=BMX055_GYRO_BW_REG;
	write_words[5]=BMX055_GYRO_BW_64; // Select Gyro BW   (32Hz)
	write_words[6]=BMX055_GYRO_LPM1_REG;
	write_words[7]=BMX055_GYRO_LPM1_MODE_NOMAL|BMX055_GYRO_LPM1_SLEEP_DUR_2MS;
	HAL_GPIO_WritePin(CS_gyro_GPIO_Port, CS_gyro_Pin, RESET);
	HAL_SPI_Transmit(hspi, write_words, 8, 2000);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CS_gyro_GPIO_Port, CS_gyro_Pin, SET);

	// magnetometer config
	write_words[0]=BMX055_RESET_REG;
	write_words[1]=BMX055_INITIATED_SOFT_RESET;
	write_words[2]=BMX055_MAG_POW_CTL_REG;
	write_words[3]=BMX055_MAG_POW_CTL_SLEEP_MODE; // Select Sleep mode
	write_words[4]=BMX055_MAG_ADV_OP_OUTPUT_REG;
	write_words[5]=BMX055_MAG_DATA_RATE_10; // (NomalMode, ODR 10Hz)
	write_words[6]=BMX055_MAG_REP_XY_REG;
	write_words[7]=0x04; // Repetitions for X-Y Axis  0x04 -> 0b00000100 -> (1+2(2^2)) = 9
	write_words[8]=BMX055_MAG_REP_Z_REG;
	write_words[9]= 0x0F; // Repetitions for Z-Axis  0x0F-> 0b00001111-> (1 +(2^0 + 2^1 + 2^2 + 2^3) = 15
	HAL_GPIO_WritePin(CS_magnet_GPIO_Port, CS_magnet_Pin, RESET);
	HAL_SPI_Transmit(hspi, write_words, 10, 2000);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CS_magnet_GPIO_Port, CS_magnet_Pin, SET);

}

void IMU_acc_read(SPI_HandleTypeDef *hspi, int *accel_data){
	uint8_t write_data=BMX055_ACC_DATA_START_REG;
	uint8_t store_data[6];

	HAL_GPIO_WritePin(CS_accel_GPIO_Port, CS_accel_Pin, RESET); //read acceloremeter data
	HAL_SPI_Transmit(hspi, &write_data, 1, 2000);
	for (int i = 0; i < 6; i++)
	{
		HAL_SPI_Receive(hspi, store_data+i, 1, 2000);
		//readI2c(accl_addr, BMX055_ACC_DATA_START_REG+i, 1, &accl_data[i]);
	}
	HAL_GPIO_WritePin(CS_accel_GPIO_Port, CS_accel_Pin, SET);
	accel_data[0]=((store_data[1]<<4)+(store_data[0]>>4));
	accel_data[1]=((store_data[3]<<4)+(store_data[2]>>4));
	accel_data[2]=((store_data[5]<<4)+(store_data[4]>>4));
	HAL_Delay(100);

	if (accel_data[0] > 2047)
	{
		accel_data[0] -= 4096;
		//accel_data[0] = accel_data[0]*0.00098;
	}
	if (accel_data[1]> 2047)
	{
		accel_data[1] -= 4096;
		//accel_data[1] = accel_data[1]*0.00098;
	}
	if (accel_data[2] > 2047)
	{
		accel_data[2] -= 4096;
		//accel_data[2] = accel_data[2]*0.00098;
	}
}

void IMU_gyro_read(SPI_HandleTypeDef *hspi, int *gyro_data){
	uint8_t write_data=BMX055_GYRO_DATA_START_REG;
	uint8_t store_data[6];

	HAL_GPIO_WritePin(CS_gyro_GPIO_Port, CS_gyro_Pin, RESET); //read acceloremeter data
	HAL_SPI_Transmit(hspi, &write_data, 1, 2000);
	for (int i = 0; i < 6; i++)
	{
		HAL_SPI_Receive(hspi, store_data+i, 1, 2000);
		//readI2c(accl_addr, BMX055_ACC_DATA_START_REG+i, 1, &accl_data[i]);
	}
	HAL_GPIO_WritePin(CS_gyro_GPIO_Port, CS_gyro_Pin, SET);
	gyro_data[0]=((store_data[1]<<8)+(store_data[0]));
	gyro_data[1]=((store_data[3]<<8)+(store_data[2]));
	gyro_data[2]=((store_data[5]<<8)+(store_data[4]));
	HAL_Delay(100);

	if (gyro_data[0] > 32767)
	{
		gyro_data[0] -= 65536;
		//accel_data[0] = accel_data[0]*0.00098;
	}
	if (gyro_data[1]> 32767)
	{
		gyro_data[1] -= 65536;
		//accel_data[1] = accel_data[1]*0.00098;
	}
	if (gyro_data[2] > 32767)
	{
		gyro_data[2] -= 65536;
		//accel_data[2] = accel_data[2]*0.00098;
	}
}

void IMU_mag_read(SPI_HandleTypeDef *hspi, int *mag_data){
	uint8_t write_data=BMX055_MAG_DATA_START_REG;
	uint8_t store_data[6];

	HAL_GPIO_WritePin(CS_magnet_GPIO_Port, CS_magnet_Pin, RESET); //read acceloremeter data
	HAL_SPI_Transmit(hspi, &write_data, 1, 2000);
	for (int i = 0; i < 6; i++)
	{
		HAL_SPI_Receive(hspi, store_data+i, 1, 2000);
			//readI2c(accl_addr, BMX055_ACC_DATA_START_REG+i, 1, &accl_data[i]);
	}
	HAL_GPIO_WritePin(CS_magnet_GPIO_Port, CS_magnet_Pin, SET);
	mag_data[0]=((store_data[1]<<5)+(store_data[0]>>3));
	mag_data[1]=((store_data[3]<<5)+(store_data[2]>>3));
	mag_data[2]=((store_data[5]<<5)+(store_data[4]>>4));

	if (mag_data[0] > 4095)
	{
		mag_data[0] -= 8192;
		//accel_data[0] = accel_data[0]*0.00098;
	}
	if (mag_data[1]> 4095)
	{
		mag_data[1] -= 8192;
			//accel_data[1] = accel_data[1]*0.00098;
	}
	if (mag_data[2] > 4095)
	{
		mag_data[2] -= 8192;
		//accel_data[2] = accel_data[2]*0.00098;
	}
}
