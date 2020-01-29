/*
 * adcUnitConversion.c
 *
 *  Created on: 27/11/2019
 */


#include "adcUnitConversion.h"
#include "stdio.h"
#include "math.h"
#include "main.h"

float adcInt2Volt (unsigned long int ADCReading){

	//ADCVREF reference voltage = ?
	return ((float)ADCReading/(1 << 12 ))*ADCVREF;
}


float motorCurrent (int adcReading){
	//0.0066 sensor's scale factor
	return (adcInt2Volt(adcReading) - BIASSENSOR)/0.0066;
}

float igbtTemp (int adcReading){

	int Rmin = 160, Rmax = 16100;
	float Vmin = 0.05, Vmax = 3.25;
	int Tmin = 0, Tmax = 150;

	float m, b, T;
	m = b = T = 0;

	m = 53.085;//(Tmin-Tmax)/(Vmax-Vmin);
	b = 22.025;//Tmin - m*Vmax;

	T = m*(adcInt2Volt(adcReading)/3.3)+b;

	return T;
}

float motorTemp (unsigned long int adcReading){

	int Rmin = 1600, Rmax = 4000;   //resistance
	float Vmin = 0.05, Vmax = 3.25; //voltage
	int Tmin=0, Tmax=150;			//temperature

	float m, b, Rt, T;
	m = b = Rt = T = 0;

	/*m = (Tmin-Tmax)/(Vmin-Vmax);
	b = Tmin-Vmin*m; */
	m = -119.81;
	b = 365.61;

	T = m*(adcInt2Volt(adcReading))+b;

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

	int R1 = 250000;
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

