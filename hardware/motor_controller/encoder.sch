EESchema Schematic File Version 4
LIBS:encoder-cache
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
Text HLabel 3150 2000 0    50   Input ~ 0
encoder
Text HLabel 3450 2000 2    50   Output ~ 0
out
$Comp
L power:+1V8 #PWR01
U 1 1 5DEFA6C6
P 3450 1700
F 0 "#PWR01" H 3450 1550 50  0001 C CNN
F 1 "+1V8" H 3465 1873 50  0000 C CNN
F 2 "" H 3450 1700 50  0001 C CNN
F 3 "" H 3450 1700 50  0001 C CNN
	1    3450 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5DEFAB95
P 3300 2000
F 0 "C1" V 3048 2000 50  0000 C CNN
F 1 "330n" V 3139 2000 50  0000 C CNN
F 2 "" H 3338 1850 50  0001 C CNN
F 3 "~" H 3300 2000 50  0001 C CNN
	1    3300 2000
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5DEFB698
P 3450 1850
F 0 "R1" H 3380 1804 50  0000 R CNN
F 1 "2M2" H 3380 1895 50  0000 R CNN
F 2 "" V 3380 1850 50  0001 C CNN
F 3 "~" H 3450 1850 50  0001 C CNN
	1    3450 1850
	-1   0    0    1   
$EndComp
$EndSCHEMATC
