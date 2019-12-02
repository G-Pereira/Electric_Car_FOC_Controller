/*includes*/
#include "main.h"
#include "FOC_lib.h"

//tamanho dos datagramas do ic 5 bytes para escrever MSB é 1


void foc_ic_config(){

	uint8_t data[5];

	data[0]= (FOC_IC_MOTOR_TYPE_N_POLE_PAIRS | 0x80); // colocar 1 no MSB do byte com o endereço
	data[1] = //o que escrever

	//colocar a zero CS do ic para iniciar comunicação por spi
	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, RESET);


	HAL_GPIO_WritePin(FOC_IC_CS_GPIO_Port, FOC_IC_CS_Pin, SET);




}
