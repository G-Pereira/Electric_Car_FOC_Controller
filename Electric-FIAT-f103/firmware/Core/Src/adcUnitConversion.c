/*
 * adcUnitConversion.c
 *
 *  Created on: 27/11/2019
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

	//considerando que a tensão corresponde linearmente a uma temperatura como no pdf da TI indicado nos desenhos

	int Rmin = 160, Rmax = 16100;
	float Vmin = 0.05, Vmax = 3.25;
	int Tmin = 0, Tmax = 150;

	float m, b, T;
	m = b = T = 0;

	m = (Tmin-Tmax)/(Vmax-Vmin);
	b = Tmin - m*Vmax;

	T = m*(adcInt2Volt(adcReading))+b;

	return T;

	/*float m = (30-115)/(1-10);
	float b = 30 - m;

	float T = m*(adcInt2Volt(adcReading))+b;

	return T;*/
}

float motorTemp (int adcReading){

	/*
	 * R1 = 1495 @ -10°C
	 * R2 = 2245 @  40°C
	 * R3 = 3817 @ 120°C
	 */


	int test = 0;

	int Rmin = 1600, Rmax = 4000;   //resistance
	float Vmin = 0.05, Vmax = 3.25; //voltage
	int Tmin=0, Tmax=150;			//temperature

	float m, b, Rt, T;
	m = b = Rt = T = 0;

	if(test){

		/*
		 * R1 = 1495 @ -10°C
		 * R2 = 2245 @  40°C
		 * R3 = 3817 @ 120°C
		 */

		float a = 0.0251239;
		float b = -0.0035737;
		float c = 0.0000123;

		/*  Divisor de tensão
		 *
		 * 	int Rk = 2700;
		 *	float Rt = Rk * ((ADCVREF/adcInt2Volt(adcReading))-1);
		*/

		m = (Rmin-Rmax)/(Vmin-Vmax);
		b = Rmin-Vmin*m;

		Rt = m*(adcInt2Volt(adcReading))+b;
		T = 1/(a+b*log(Rt)+c*pow(log(Rt),3));


	} else {

		//considerando que a tensÃ£o corresponde linearmente a uma temperatura como no pdf da TI indicado nos desenhos

		m = (Tmin-Tmax)/(Vmin-Vmax);
		b = Tmin-Vmin*m;

		T = m*(adcInt2Volt(adcReading))+b;

	}


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

float voltageDC (int adcReading){

	/*
	 * R1 = R9 = 300k
	 * R2 = R12 = 2k
	 * R3 = R10 = 39  <- negligenciar?
	 */

	int R1 = 300000;
	int R2 = 2000;
	int R3 = 39;

	float Vout = adcInt2Volt(adcReading);

	float Vin = ((R1*R2)/R2)*Vout;

	return Vin;

}


float pedalPos (int adcReading){

	float pos = (adcInt2Volt(adcReading)/ADCVREF)*100;

	return pos;

}

