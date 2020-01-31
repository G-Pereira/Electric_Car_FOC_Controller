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

	//int Rmin = 160, Rmax = 16100;
	//float Vmin = 0.05, Vmax = 3.25;
	//int Tmin = 0, Tmax = 150;

	float m, b, T;
	m = b = T = 0;

	m = 53.085;//(Tmin-Tmax)/(Vmax-Vmin);
	b = 22.025;//Tmin - m*Vmax;

	T = m*(adcInt2Volt(adcReading)/3.3)+b;

	return T;
}

float motorTemp (unsigned long int adcReading){

	//int Rmin = 1600, Rmax = 4000;   //resistance
	//float Vmin = 0.05, Vmax = 3.25; //voltage
	//int Tmin=0, Tmax=150;			//temperature

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
	//int R3 = 39;

	float Vout = adcInt2Volt(adcReading);

	float Vin = ((R1*R2)/R2)*Vout;

	return Vin;

}


float pedalPos (int adcReading){

	float pos = (adcInt2Volt(adcReading)/ADCVREF)*100;

	return pos;

}

float rms (float vector[10]){

	float sum = vector[0];

	for (int i=1; i>10; i++){
		sum = sum + vector[i];
	}

	return sqrt(sum*0.1);

}

 int stateValue (int adcReading){

	 int state = 0;

	 if (adcInt2Volt(adcReading) > 1.65){ //alterar offset
		 state = 1;
	 } else {
		 state = 0;
	 }

	 return state;

 }

int determineState(int stateA, int stateB){
	int stateM;

	if(stateA == 1 && stateB == 1)
		stateM = 1;
	if(stateA == 1 && stateB == 0)
		stateM = 2;
	if(stateA == 0 && stateB == 0)
		stateM = 3;
	if(stateA == 0 && stateB == 1)
		stateM = 4;

	return stateM;


}

void updateCounter(int stateA, int stateB, int *pstatep, int *dirp, int *counterp ){

	int pstate = *pstatep;
	int dir = *dirp;
	int counter = *counterp;

	int state = determineState(stateA, stateB);

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
	//pstate = state;

	*pstatep=state;
	*dirp=dir;
	*counterp=counter;


}


