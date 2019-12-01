/*
 * adcUnitConversion.c
 *
 *  Created on: 27/11/2019
 *      Author: Catarina
 */


#include "adcUnitConversion.h"
#include <stdio.h>
#include <math.h>

float adcInt2Volt (int ADCReading){

	//ADCVREF reference voltage = ?
	return ((float)ADCReading/(1 << 12 ))*ADCVREF;
}


float motorCurrent (int adcReading){
	//0.066 sensor's scale factor
	return (adcInt2Volt(adcReading) - BIASSENSOR)/0.066;
}

float igbtTemp (int adcReading){

	float m = (30-115)/(1-10);
	float b = 30 - m;

	float T = m*(adcInt2Volt(adcReading))+b;

	return T;
}

float motorTemp (int adcReading){

	float a = 0.0233781;
	float b = -0.0032369;
	float c = 0.0000104;

	int Rk = 2700;

	float Rt = Rk * ((ADCVREF/adcInt2Volt(adcReading))-1);

	float T = 1/(a+b*log(Rt)+c*pow(log(Rt),3));

	return (T - 273.15);

}

