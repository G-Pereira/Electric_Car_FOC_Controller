/*includes*/
#include "main.h"
#include "FOC_lib.h"
#include "stdio.h"

// use TMC4671 API

//tamanho dos datagramas do ic 5 bytes para escrever MSB é 1

void TMC_get_data(uint8_t *data, uint32_t w_data){
	data[0]=(uint8_t)(w_data>>24);
	data[1]=(uint8_t)(w_data>>16);
	data[2]=(uint8_t)(w_data>>8);
	data[3]=(uint8_t)(w_data);

}

void TMC_write(SPI_HandleTypeDef *hspi, uint8_t address, uint32_t data){
	uint8_t send_data[5];


	send_data[0]= (address | 0x80);
	send_data[1]=(uint8_t)(data>>24);
	send_data[2]=(uint8_t)(data>>16);
	send_data[3]=(uint8_t)(data>>8);
	send_data[4]=(uint8_t)(data | 0x00);
	printf("%d - %d - %d - %d - %d" , send_data[0], send_data[1], send_data[2], send_data[3], send_data[4]);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, RESET);
	HAL_SPI_Transmit(hspi, send_data, 5, 200);
	HAL_GPIO_WritePin(SPI_CS_FOC_GPIO_Port, SPI_CS_FOC_Pin, SET);

}


void foc_ic_config(SPI_HandleTypeDef *hspi){

	uint8_t data[4]; //32 bit cada registo

	//data[0]= ((FOC_IC_MOTOR_TYPE_N_POLE_PAIRS | 0x80)<<8); // colocar 1 no MSB do byte com o endereço
	//data[1] = //o que escrever?

	// Motor type &  PWM configuration
	TMC_write(hspi, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x0003000A);
	TMC_write(hspi, TMC4671_PWM_POLARITIES, 0x00000000);
	TMC_write(hspi, TMC4671_PWM_MAXCNT, 0x00000F9F);
	TMC_write(hspi, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
	TMC_write(hspi, TMC4671_PWM_SV_CHOP, 0x00000107);

			// ADC configuration
	TMC_write(hspi, TMC4671_ADC_I_SELECT, 0x09000100);
	TMC_write(hspi, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
	TMC_write(hspi, TMC4671_dsADC_MCLK_A, 0x20000000);
	TMC_write(hspi, TMC4671_dsADC_MCLK_B, 0x20000000);
	TMC_write(hspi, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
	TMC_write(hspi, TMC4671_ADC_I0_SCALE_OFFSET, 0xFF006D20);
	TMC_write(hspi, TMC4671_ADC_I1_SCALE_OFFSET, 0xFF006C62);

			// Open loop settings
	TMC_write(hspi, TMC4671_OPENLOOP_MODE, 0x00000000);
	TMC_write(hspi, TMC4671_OPENLOOP_ACCELERATION, 20);
	TMC_write(hspi, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);//0xFFFFFFFB);

			// Feedback selection
	TMC_write(hspi, TMC4671_PHI_E_SELECTION, 0x00000002);
	TMC_write(hspi, TMC4671_UQ_UD_EXT, 6000);

			// ===== Open loop test drive =====

			// Switch to open loop velocity mode
	TMC_write(hspi, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

			// Rotate right
	TMC_write(hspi, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000003C);
			HAL_Delay(200);

			// Rotate left
	TMC_write(hspi, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFC4);
			HAL_Delay(400);

			// Stop
    TMC_write(hspi, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);


}


void foc_ic_send_torque(SPI_HandleTypeDef *hspi, int torque, float pos){


	uint8_t data[4];
	uint32_t aux;

	if (pos!=-1){
		torque_convertion(torque, data, pos);
	} else {
		//memset(data, 0, 4);
		data[0]=0;
		data[1]=0;
		data[2]=0;
		data[3]=0;
	}

	aux=(uint32_t)(data[0]<<24 | data[1]<<16 | data[2] << 8 | data[3]);
	TMC_write(hspi, TMC4671_OPENLOOP_VELOCITY_TARGET, aux);


}

void torque_convertion(int torque, uint8_t* torque_ref, float pos){

	//uint8_t torque_ref[4];

	int max_ref = 120;

	uint32_t aux = (pos/100)*max_ref;
	TMC_get_data(torque_ref, aux);


}



