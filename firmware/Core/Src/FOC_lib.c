/*includes*/
#include "main.h"
#include "FOC_lib.h"

//tamanho dos datagramas do ic 5 bytes para escrever MSB é 1


void foc_ic_config(SPI_HandleTypeDef *hspi){

	uint8_t data[4]; //32 bit cada registo

	data[0]= ((FOC_IC_MOTOR_TYPE_N_POLE_PAIRS | 0x80)<<8); // colocar 1 no MSB do byte com o endereço
	//data[1] = //o que escrever

	//colocar a zero CS do ic para iniciar comunicação por spi

	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);


	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);


	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);


	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);

}


void foc_ic_send_torque(SPI_HandleTypeDef *hspi, int torque){


	uint8_t* data;

	data=torque_convertion(torque);
	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 2, 200);
	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);


}
