EESchema Schematic File Version 4
LIBS:electric-FIAT-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 12 13
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
P 5800 4250
AR Path="/5DEDB0CD/5DE55E17" Ref="Q1"  Part="1" 
AR Path="/5DEE527A/5DE55E17" Ref="Q2"  Part="1" 
AR Path="/5DEF4ACC/5DE55E17" Ref="Q3"  Part="1" 
AR Path="/5DF13C30/5DE55E17" Ref="Q4"  Part="1" 
AR Path="/5DF13C34/5DE55E17" Ref="Q5"  Part="1" 
AR Path="/5DF13C38/5DE55E17" Ref="Q6"  Part="1" 
F 0 "Q6" V 6051 4250 50  0000 C CNN
F 1 "AO3401A" V 6142 4250 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6000 4175 50  0001 L CIN
F 3 "http://www.aosmd.com/pdfs/datasheet/AO3401A.pdf" H 5800 4250 50  0001 L CNN
	1    5800 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R R36
U 1 1 5DE572D3
P 5400 4150
AR Path="/5DEDB0CD/5DE572D3" Ref="R36"  Part="1" 
AR Path="/5DEE527A/5DE572D3" Ref="R38"  Part="1" 
AR Path="/5DEF4ACC/5DE572D3" Ref="R40"  Part="1" 
AR Path="/5DF13C30/5DE572D3" Ref="R42"  Part="1" 
AR Path="/5DF13C34/5DE572D3" Ref="R44"  Part="1" 
AR Path="/5DF13C38/5DE572D3" Ref="R46"  Part="1" 
F 0 "R46" H 5470 4196 50  0000 L CNN
F 1 "1k" H 5470 4105 50  0000 L CNN
F 2 "" V 5330 4150 50  0001 C CNN
F 3 "~" H 5400 4150 50  0001 C CNN
	1    5400 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R37
U 1 1 5DE58624
P 6100 4150
AR Path="/5DEDB0CD/5DE58624" Ref="R37"  Part="1" 
AR Path="/5DEE527A/5DE58624" Ref="R39"  Part="1" 
AR Path="/5DEF4ACC/5DE58624" Ref="R41"  Part="1" 
AR Path="/5DF13C30/5DE58624" Ref="R43"  Part="1" 
AR Path="/5DF13C34/5DE58624" Ref="R45"  Part="1" 
AR Path="/5DF13C38/5DE58624" Ref="R47"  Part="1" 
F 0 "R47" H 6170 4196 50  0000 L CNN
F 1 "1k" H 6170 4105 50  0000 L CNN
F 2 "" V 6030 4150 50  0001 C CNN
F 3 "~" H 6100 4150 50  0001 C CNN
	1    6100 4150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR090
U 1 1 5DE587FC
P 5400 3900
AR Path="/5DEDB0CD/5DE587FC" Ref="#PWR090"  Part="1" 
AR Path="/5DEE527A/5DE587FC" Ref="#PWR092"  Part="1" 
AR Path="/5DEF4ACC/5DE587FC" Ref="#PWR094"  Part="1" 
AR Path="/5DF13C30/5DE587FC" Ref="#PWR096"  Part="1" 
AR Path="/5DF13C34/5DE587FC" Ref="#PWR098"  Part="1" 
AR Path="/5DF13C38/5DE587FC" Ref="#PWR0100"  Part="1" 
F 0 "#PWR0100" H 5400 3750 50  0001 C CNN
F 1 "+3.3V" H 5415 4073 50  0000 C CNN
F 2 "" H 5400 3900 50  0001 C CNN
F 3 "" H 5400 3900 50  0001 C CNN
	1    5400 3900
	1    0    0    -1  
$EndComp
Text HLabel 5250 4350 0    50   Input ~ 0
in
Text HLabel 6250 4350 2    50   Output ~ 0
out
Wire Wire Line
	5250 4350 5400 4350
Wire Wire Line
	5400 4000 5400 3950
Wire Wire Line
	5400 4300 5400 4350
Connection ~ 5400 4350
Wire Wire Line
	5400 4350 5600 4350
Wire Wire Line
	5800 3950 5400 3950
Wire Wire Line
	5800 3950 5800 4050
Connection ~ 5400 3950
Wire Wire Line
	5400 3950 5400 3900
Wire Wire Line
	6000 4350 6100 4350
Wire Wire Line
	6100 4300 6100 4350
Connection ~ 6100 4350
Wire Wire Line
	6100 4350 6250 4350
Wire Wire Line
	6100 3900 6100 4000
$Comp
L power:+15V #PWR0113
U 1 1 5DEB8F65
P 6100 3900
AR Path="/5DF13C38/5DEB8F65" Ref="#PWR0113"  Part="1" 
AR Path="/5DF13C34/5DEB8F65" Ref="#PWR0112"  Part="1" 
AR Path="/5DEDB0CD/5DEB8F65" Ref="#PWR0108"  Part="1" 
AR Path="/5DEE527A/5DEB8F65" Ref="#PWR0109"  Part="1" 
AR Path="/5DEF4ACC/5DEB8F65" Ref="#PWR0110"  Part="1" 
AR Path="/5DF13C30/5DEB8F65" Ref="#PWR0111"  Part="1" 
F 0 "#PWR0113" H 6100 3750 50  0001 C CNN
F 1 "+15V" H 6115 4073 50  0000 C CNN
F 2 "" H 6100 3900 50  0001 C CNN
F 3 "" H 6100 3900 50  0001 C CNN
	1    6100 3900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
