EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 7
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	6250 3650 6250 3700
$Comp
L power:GND #PWR?
U 1 1 5DC6FAA7
P 6250 4350
AR Path="/5DC7202D/5DC6FAA7" Ref="#PWR?"  Part="1" 
AR Path="/5DC75544/5DC6FAA7" Ref="#PWR?"  Part="1" 
AR Path="/5DC755EE/5DC6FAA7" Ref="#PWR?"  Part="1" 
AR Path="/5DC773B5/5DC6FAA7" Ref="#PWR04"  Part="1" 
AR Path="/5DC77C5E/5DC6FAA7" Ref="#PWR09"  Part="1" 
AR Path="/5DC77D5C/5DC6FAA7" Ref="#PWR014"  Part="1" 
F 0 "#PWR014" H 6250 4100 50  0001 C CNN
F 1 "GND" H 6255 4177 50  0000 C CNN
F 2 "" H 6250 4350 50  0001 C CNN
F 3 "" H 6250 4350 50  0001 C CNN
	1    6250 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4350 6250 4300
Text HLabel 5850 3900 0    50   Input ~ 0
IGBT_PH1
Text HLabel 5850 4100 0    50   Output ~ 0
Motor_PH1
Text Notes 4100 3550 0    100  ~ 20
Decoupling
$Comp
L Device:C C?
U 1 1 5DC71A7F
P 4500 4100
AR Path="/5DC7202D/5DC71A7F" Ref="C?"  Part="1" 
AR Path="/5DC75544/5DC71A7F" Ref="C?"  Part="1" 
AR Path="/5DC755EE/5DC71A7F" Ref="C?"  Part="1" 
AR Path="/5DC773B5/5DC71A7F" Ref="C1"  Part="1" 
AR Path="/5DC77C5E/5DC71A7F" Ref="C3"  Part="1" 
AR Path="/5DC77D5C/5DC71A7F" Ref="C5"  Part="1" 
F 0 "C5" H 4615 4146 50  0000 L CNN
F 1 "100n" H 4615 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4538 3950 50  0001 C CNN
F 3 "~" H 4500 4100 50  0001 C CNN
	1    4500 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5DC71F2C
P 4500 3900
AR Path="/5DC7202D/5DC71F2C" Ref="#PWR?"  Part="1" 
AR Path="/5DC75544/5DC71F2C" Ref="#PWR?"  Part="1" 
AR Path="/5DC755EE/5DC71F2C" Ref="#PWR?"  Part="1" 
AR Path="/5DC773B5/5DC71F2C" Ref="#PWR02"  Part="1" 
AR Path="/5DC77C5E/5DC71F2C" Ref="#PWR07"  Part="1" 
AR Path="/5DC77D5C/5DC71F2C" Ref="#PWR012"  Part="1" 
F 0 "#PWR012" H 4500 3750 50  0001 C CNN
F 1 "+3.3V" H 4515 4073 50  0000 C CNN
F 2 "" H 4500 3900 50  0001 C CNN
F 3 "" H 4500 3900 50  0001 C CNN
	1    4500 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 3900 4500 3950
$Comp
L power:GND #PWR?
U 1 1 5DC7230E
P 4500 4300
AR Path="/5DC7202D/5DC7230E" Ref="#PWR?"  Part="1" 
AR Path="/5DC75544/5DC7230E" Ref="#PWR?"  Part="1" 
AR Path="/5DC755EE/5DC7230E" Ref="#PWR?"  Part="1" 
AR Path="/5DC773B5/5DC7230E" Ref="#PWR03"  Part="1" 
AR Path="/5DC77C5E/5DC7230E" Ref="#PWR08"  Part="1" 
AR Path="/5DC77D5C/5DC7230E" Ref="#PWR013"  Part="1" 
F 0 "#PWR013" H 4500 4050 50  0001 C CNN
F 1 "GND" H 4505 4127 50  0000 C CNN
F 2 "" H 4500 4300 50  0001 C CNN
F 3 "" H 4500 4300 50  0001 C CNN
	1    4500 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4300 4500 4250
$Comp
L Device:C C?
U 1 1 5DC730E7
P 6750 4200
AR Path="/5DC7202D/5DC730E7" Ref="C?"  Part="1" 
AR Path="/5DC75544/5DC730E7" Ref="C?"  Part="1" 
AR Path="/5DC755EE/5DC730E7" Ref="C?"  Part="1" 
AR Path="/5DC773B5/5DC730E7" Ref="C2"  Part="1" 
AR Path="/5DC77C5E/5DC730E7" Ref="C4"  Part="1" 
AR Path="/5DC77D5C/5DC730E7" Ref="C6"  Part="1" 
F 0 "C6" H 6865 4246 50  0000 L CNN
F 1 "1n" H 6865 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6788 4050 50  0001 C CNN
F 3 "~" H 6750 4200 50  0001 C CNN
	1    6750 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DC7443D
P 6750 4400
AR Path="/5DC7202D/5DC7443D" Ref="#PWR?"  Part="1" 
AR Path="/5DC75544/5DC7443D" Ref="#PWR?"  Part="1" 
AR Path="/5DC755EE/5DC7443D" Ref="#PWR?"  Part="1" 
AR Path="/5DC773B5/5DC7443D" Ref="#PWR05"  Part="1" 
AR Path="/5DC77C5E/5DC7443D" Ref="#PWR010"  Part="1" 
AR Path="/5DC77D5C/5DC7443D" Ref="#PWR015"  Part="1" 
F 0 "#PWR015" H 6750 4150 50  0001 C CNN
F 1 "GND" H 6755 4227 50  0000 C CNN
F 2 "" H 6750 4400 50  0001 C CNN
F 3 "" H 6750 4400 50  0001 C CNN
	1    6750 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 4400 6750 4350
Wire Wire Line
	6750 4050 6750 4000
Wire Wire Line
	6750 4000 6650 4000
Text HLabel 6750 4000 2    50   Output ~ 0
out
$Comp
L power:+3V3 #PWR035
U 1 1 5DCEB1D3
P 6250 3650
AR Path="/5DC773B5/5DCEB1D3" Ref="#PWR035"  Part="1" 
AR Path="/5DC77C5E/5DCEB1D3" Ref="#PWR036"  Part="1" 
AR Path="/5DC77D5C/5DCEB1D3" Ref="#PWR037"  Part="1" 
F 0 "#PWR037" H 6250 3500 50  0001 C CNN
F 1 "+3V3" H 6265 3823 50  0000 C CNN
F 2 "" H 6250 3650 50  0001 C CNN
F 3 "" H 6250 3650 50  0001 C CNN
	1    6250 3650
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Current:ACS759xCB-200B-PFF U2
U 1 1 5DCAFD9E
P 6250 4000
AR Path="/5DC773B5/5DCAFD9E" Ref="U2"  Part="1" 
AR Path="/5DC77C5E/5DCAFD9E" Ref="U3"  Part="1" 
AR Path="/5DC77D5C/5DCAFD9E" Ref="U7"  Part="1" 
F 0 "U7" H 6350 4350 50  0000 L CNN
F 1 "ACS759xCB-200B-PFF" H 6350 4250 50  0000 L CNN
F 2 "Sensor_Current:Allegro_CB_PFF" H 6250 4000 50  0001 C CNN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/ACS759-Datasheet.ashx?la=en" H 6250 4000 50  0001 C CNN
	1    6250 4000
	1    0    0    -1  
$EndComp
$EndSCHEMATC