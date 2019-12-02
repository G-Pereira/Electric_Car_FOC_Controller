EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Transistor_FET:AO3401A Q1
U 1 1 5DE55E17
P 5500 3950
F 0 "Q1" V 5751 3950 50  0000 C CNN
F 1 "AO3401A" V 5842 3950 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5700 3875 50  0001 L CIN
F 3 "http://www.aosmd.com/pdfs/datasheet/AO3401A.pdf" H 5500 3950 50  0001 L CNN
	1    5500 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5DE572D3
P 5000 3600
F 0 "R1" H 5070 3646 50  0000 L CNN
F 1 "1k" H 5070 3555 50  0000 L CNN
F 2 "" V 4930 3600 50  0001 C CNN
F 3 "~" H 5000 3600 50  0001 C CNN
	1    5000 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5DE58624
P 5950 3600
F 0 "R2" H 6020 3646 50  0000 L CNN
F 1 "1k" H 6020 3555 50  0000 L CNN
F 2 "" V 5880 3600 50  0001 C CNN
F 3 "~" H 5950 3600 50  0001 C CNN
	1    5950 3600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5DE587FC
P 5000 3450
F 0 "#PWR01" H 5000 3300 50  0001 C CNN
F 1 "+3.3V" H 5015 3623 50  0000 C CNN
F 2 "" H 5000 3450 50  0001 C CNN
F 3 "" H 5000 3450 50  0001 C CNN
	1    5000 3450
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR02
U 1 1 5DE58E01
P 5950 3450
F 0 "#PWR02" H 5950 3300 50  0001 C CNN
F 1 "+24V" H 5965 3623 50  0000 C CNN
F 2 "" H 5950 3450 50  0001 C CNN
F 3 "" H 5950 3450 50  0001 C CNN
	1    5950 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3750 5000 4050
Wire Wire Line
	5000 4050 5300 4050
Wire Wire Line
	5700 4050 5950 4050
Wire Wire Line
	5950 4050 5950 3750
Wire Wire Line
	5500 3750 5500 3450
Wire Wire Line
	5500 3450 5000 3450
Connection ~ 5000 3450
Text HLabel 4900 4050 0    50   Input ~ 0
FOC
Wire Wire Line
	4900 4050 5000 4050
Connection ~ 5000 4050
Text HLabel 6050 4050 2    50   Input ~ 0
IGBT
Wire Wire Line
	6050 4050 5950 4050
Connection ~ 5950 4050
$EndSCHEMATC
