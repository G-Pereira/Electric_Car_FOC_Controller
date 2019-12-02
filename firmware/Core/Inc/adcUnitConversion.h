/*
 * adcUnitConversion.h
 *
 *  Created on: 27/11/2019
 *      Author: Catarina
 */

#ifndef INC_ADCUNITCONVERSION_H_
#define INC_ADCUNITCONVERSION_H_

#define BIASSENSOR 1.65
#define ADCVREF 3.3

float adcInt2Volt (int ADCReading);

float motorCurrent (int adcReading);
float igbtTemp (int adcReading);
float motorTemp (int adcReading);

float voltageAC (int adcReading);

#endif /* INC_ADCUNITCONVERSION_H_ */
