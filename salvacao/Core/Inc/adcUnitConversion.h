/*
 * adcUnitConversion.h
 *
 *  Created on: 27/11/2019
 */

#ifndef INC_ADCUNITCONVERSION_H_
#define INC_ADCUNITCONVERSION_H_

#define BIASSENSOR 1.65
#define ADCVREF 3.3

float adcInt2Volt (unsigned long int ADCReading);

float motorCurrent (int adcReading);
float igbtTemp (int adcReading);
float motorTemp (unsigned long int adcReading);

float voltageAC (int adcReading);
float voltageDC (int adcReading);

float pedalPos (int adcReading);

float rms (float vector[10]); //convert instantaneous current/voltage to their rms value

int stateValue (int adcReading); //"schmitt trigger"
int determineState(int stateA, int stateB);
void updateCounter(int stateA, int stateB, int *pstate, int *dir, int *pulses);

#endif /* INC_ADCUNITCONVERSION_H_ */
