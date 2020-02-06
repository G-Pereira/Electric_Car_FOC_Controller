/*includes*/
#include "main.h"
#include "FOC_lib.h"

//tamanho dos datagramas do ic 5 bytes para escrever MSB é 1


void TMC_write(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t *data){
	uint8_t send_data[5];

	send_data[0]= (address | 0x80);
	send_data[1]=data[0];
	send_data[2]=data[1];
	send_data[3]=data[2];
	send_data[4]=data[3];

	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
	HAL_SPI_Transmit(hspi, send_data, 5, 200);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);

}


void foc_ic_config(SPI_HandleTypeDef *hspi){

	uint8_t data[4]; //32 bit cada registo

	//data[0]= ((FOC_IC_MOTOR_TYPE_N_POLE_PAIRS | 0x80)<<8); // colocar 1 no MSB do byte com o endereço
	//data[1] = //o que escrever?

	//colocar a zero CS do ic para iniciar comunicação por spi

	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);


	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);


	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);


	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 4, 200);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);

}


void foc_ic_send_torque(SPI_HandleTypeDef *hspi, int torque){


	uint8_t* data;

	data=torque_convertion(torque);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
		HAL_SPI_Transmit(hspi, data, 2, 200);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);


}

uint8_t *torque_convertion(int torque){

	uint8_t torque_ref[4];


	return torque_ref;//??
}

