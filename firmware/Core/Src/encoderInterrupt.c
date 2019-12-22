/*
 * encoderInterrupt.c
 *
 *  Created on: Dec 22, 2019
 */

#include "encoderInterrupt.h"

void stateSetup((int *pstate, uint32_t *tick)){

	*tick = HAL_GetTick();

	A = HAL_GPIO_ReadPin(GPIOA, encoder_va_Pin);
	B = HAL_GPIO_ReadPin(GPIOA, encoder_vb_Pin);
	*pstate = determineState(A,B);

}

int determineState(GPIO_PinState A, GPIO_PinState B){

	if((A==GPIO_PIN_SET)&&(B==GPIO_PIN_SET))
		state = 1;
	if((A==GPIO_PIN_SET)&&(B==GPIO_PIN_RESET))
		state = 2;
	if((A==GPIO_PIN_RESET)&&(B==GPIO_PIN_RESET))
		state = 3;
	if((A==GPIO_PIN_RESET)&&(B==GPIO_PIN_SET))
		state = 4;

	return state;
}


void updateCounter(GPIO_PinState A, GPIO_PinState B, int *pstatep, int *dirp, int *counterp){

	int pstate = *pstatep;
	int dir = *dirp;
	int counter = *counterp;

	int state = determineState(A,B);

	switch(state){
	  case 1:
		  if (pstate == 2)
			  dir = -1;
		  if (pstate == 4)
			  dir = 0;
		  break;
	  case 2:
		  if (pstate == 1)
			  dir = 0;
		  if (pstate == 3)
			  dir = -1;
		  break;
	  case 3:
		  if (pstate == 2)
			  dir = 0;
		  if (pstate == 4)
			  dir = -1;
		  break;
	  case 4:
		  if(pstate == 1)
			  dir = -1;
		  if(pstate == 4)
			  dir = 0;
	}

	counter++;
	pstate = state;

	*pstatep = pstate;
	*dirp=dir;
	*counterp=counter;

}

int rpm (uint32_t *tick){

	uint32_t interval = HAL_GetTick()-*tick;

	int rpm = ((counter/8)*60)/interval;

	counter = 0;
	*tick = HAL_GetTick();

	return rpm;

}
