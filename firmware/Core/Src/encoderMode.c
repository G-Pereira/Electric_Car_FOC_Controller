/*
 * encoderMode.c
 *
 *  Created on: 18/12/2019
 */


#include "encoderMode.h"
#include <tim.h>

float motorSpeed (uint32_t cnt1){

	uint32_t cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t diff = 0;

	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
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

	uint32_t speed = (diff/8)*60;  // change when sysTimer configured

	return speed;
}
