#ifndef _FOC_LIB_H_
#define _FOC_LIB_H_


	#define FOC_IC_MOTOR_TYPE_N_POLE_PAIRS 0x1B
	#define FOC_IC_N_POLE_PAIRS
	#define FOC_IC_BBM_TIMES  //TEMPO M�?XIMO COMUTAÇÃO DOS IGBTS
	#define FOC_IC_PWM_POLARITIES_1  //0 NOS DOIS REGISTOS
	#define FOC_IC_PWM_POLARITIES_2
	#define FOC_IC_PWM_MAX_COUNT  //(100MHz/FREQ_COMU)-1
	#define FOC_IC_PWM_CHOP  // COLOCAR A 7
	#define FOC_IC_PWM_SV
	#define FOC_IC_PHI_E_SELECTION
	#define FOC_IC_MODE_MOTION


	#define FOC_IC_TORQUE_REF  //endereço para enviar referência de binário


void foc_ic_config(SPI_HandleTypeDef *hspi);

void foc_ic_send_torque(SPI_HandleTypeDef *hspi, int torque);

uint8_t * torque_convertion(int torque);


#endif
