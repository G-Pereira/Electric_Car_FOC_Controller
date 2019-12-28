/*
 * encoderMode.c
 *
 *  Created on: 18/12/2019
 */


#include "main.h"
#include "encoderMode.h"


float motorSpeed (uint32_t cnt1, TIM_HandleTypeDef htim3){

	uint32_t cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
	int diff = 0;

	printf("counter2 = %lu\n",cnt2);

	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
		if(cnt2 < cnt1)
			diff = cnt1 - cnt2;
		else
			diff = (65535 - cnt2) + cnt1;
	} else {
		if(cnt2 > cnt1)
			diff = cnt2 - cnt1;
		else
			diff = (65535 - cnt1) + cnt2;
	}



	printf("diff = %d\n", diff);
	float speed = ((diff/4)*60);  // change when sysTimer configured
	//printf("aqui %f\n",speed);
	return speed*2;
}
