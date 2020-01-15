/*
 * encoderMode.c
 *
 *  Created on: 18/12/2019
 */

#include "main.h"
#include "encoderMode.h"

int motorSpeed (uint32_t *cnt1, uint32_t *tick, TIM_HandleTypeDef htim2){

	uint32_t cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
	int diff = 0;

	printf("cnt1 = %lu counter2 = %lu\n",*cnt1,cnt2);

	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
		if(cnt2 < *cnt1)
			diff = *cnt1 - cnt2;
		else
			diff = (65535 - cnt2) + *cnt1;
	} else {
		if(cnt2 > *cnt1)
			diff = cnt2 - *cnt1;
		else
			diff = (65535 - *cnt1) + cnt2;
	}



	printf("diff = %d\n", diff);
	//float speed = (((float)(diff)/8)*60)*2;  //((diff/8)*60)/0.02;  // change when sysTimer configured
	float speed = (((float)(diff)/4)*60)/(HAL_GetTick() - *tick)*0.001;


	*tick = HAL_GetTick();
	*cnt1 = cnt2;
	//printf("aqui %f\n",speed);
	return speed;
}
