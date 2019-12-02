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
	//0.0066 sensor's scale factor
	return (adcInt2Volt(adcReading) - BIASSENSOR)/0.0066;
}

float igbtTemp (int adcReading){

	float m = (30-115)/(1-10);
	float b = 30 - m;

	float T = m*(adcInt2Volt(adcReading))+b;

	return T;
}

float motorTemp (int adcReading){

	/*
	 * R1 = 1495 @ -10°C
	 * R2 = 2245 @  40°C
	 * R3 = 3817 @ 120°C
	 */

	float a = 0.0251239;
	float b = -0.0035737;
	float c = 0.0000123;

	int Rk = 2700;

	float Rt = Rk * ((ADCVREF/adcInt2Volt(adcReading))-1);

	float T = 1/(a+b*log(Rt)+c*pow(log(Rt),3));

	return T;

}

float voltageAC (int adcReading){

	/*
	 * R1 = R28 = 75k
	 * R2 = R30 = 10k
	 */
	int R1 = 75000;
	int R2 = 10000;

	float Vout = adcInt2Volt(adcReading);

	float Vin = ((R1*R2)/R2)*Vout;

	return Vin;

}

