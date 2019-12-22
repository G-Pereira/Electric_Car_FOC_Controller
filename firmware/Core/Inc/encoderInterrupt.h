/*
 * encoderInterrupt.h
 *
 *  Created on: Dec 22, 2019
 */

#ifndef INC_ENCODERINTERRUPT_H_
#define INC_ENCODERINTERRUPT_H_


void stateSetup(int *pstate, uint32_t tick);
int determineState(GPIO_PinState A, GPIO_PinState B);
void updateCounter(GPIO_PinState A, GPIO_PinState B, int *pstatep, int *dirp, int *counterp);

int rpm (uint32_t *tick);



#endif /* INC_ENCODERINTERRUPT_H_ */
