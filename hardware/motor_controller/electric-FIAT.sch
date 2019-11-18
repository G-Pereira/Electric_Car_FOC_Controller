EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 7
Title "Motor Controller"
Date "2019-11-18"
Rev "0.3"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 14100 8100 0    100  ~ 20
AC Current Sensors
$Sheet
S 14350 8300 800  300 
U 5DC773B5
F0 "Current Sensor Phase 1" 50
F1 "current_sensor.sch" 50
F2 "IGBT_PH1" I L 14350 8400 50 
F3 "Motor_PH1" O L 14350 8500 50 
F4 "out" O R 15150 8400 50 
$EndSheet
$Sheet
S 14350 8900 800  300 
U 5DC77C5E
F0 "Current Sensor Phase 2" 50
F1 "current_sensor.sch" 50
F2 "IGBT_PH1" I L 14350 9000 50 
F3 "Motor_PH1" O L 14350 9100 50 
F4 "out" O R 15150 9000 50 
$EndSheet
$Sheet
S 14350 9500 800  300 
U 5DC77D5C
F0 "Current Sensor Phase 3" 50
F1 "current_sensor.sch" 50
F2 "IGBT_PH1" I L 14350 9600 50 
F3 "Motor_PH1" O L 14350 9700 50 
F4 "out" O R 15150 9600 50 
$EndSheet
$Comp
L Device:R R4
U 1 1 5DBCC022
P 12650 3000
F 0 "R4" V 12443 3000 50  0000 C CNN
F 1 "3.3k" V 12534 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 12580 3000 50  0001 C CNN
F 3 "~" H 12650 3000 50  0001 C CNN
	1    12650 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	13050 3000 13050 2100
Wire Wire Line
	12800 3000 13050 3000
Wire Wire Line
	12650 1750 12650 1800
$Comp
L power:GND #PWR022
U 1 1 5DBD246A
P 12650 2450
F 0 "#PWR022" H 12650 2200 50  0001 C CNN
F 1 "GND" H 12655 2277 50  0000 C CNN
F 2 "" H 12650 2450 50  0001 C CNN
F 3 "" H 12650 2450 50  0001 C CNN
	1    12650 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	12650 2450 12650 2400
Wire Wire Line
	12500 3000 12350 3000
Wire Wire Line
	12350 3000 12350 2200
Wire Wire Line
	12350 2200 12450 2200
$Comp
L power:+1V8 #PWR024
U 1 1 5DBD45BD
P 11800 2950
F 0 "#PWR024" H 11800 2800 50  0001 C CNN
F 1 "+1V8" H 11815 3123 50  0000 C CNN
F 2 "" H 11800 2950 50  0001 C CNN
F 3 "" H 11800 2950 50  0001 C CNN
	1    11800 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5DBD5C6F
P 12100 3000
F 0 "R3" V 11893 3000 50  0000 C CNN
F 1 "1k" V 11984 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 12030 3000 50  0001 C CNN
F 3 "~" H 12100 3000 50  0001 C CNN
	1    12100 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	11950 3000 11800 3000
Wire Wire Line
	11800 3000 11800 2950
Wire Wire Line
	12250 3000 12350 3000
Connection ~ 12350 3000
$Comp
L Sensor_Temperature:KTY81 TH1
U 1 1 5DBD769F
P 11800 1800
F 0 "TH1" H 11897 1846 50  0000 L CNN
F 1 "KTY81" H 11897 1755 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00012_1x02_P5.00mm_Horizontal" V 12000 1800 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/KTY81_SER.pdf" H 11800 1750 50  0001 C CNN
	1    11800 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5DBD9F8E
P 11800 2200
F 0 "R1" H 11870 2246 50  0000 L CNN
F 1 "2.7k" H 11870 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 11730 2200 50  0001 C CNN
F 3 "~" H 11800 2200 50  0001 C CNN
	1    11800 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	11800 1950 11800 2000
Connection ~ 11800 2000
Wire Wire Line
	11800 2000 11800 2050
$Comp
L power:GND #PWR020
U 1 1 5DBDC474
P 11800 2400
F 0 "#PWR020" H 11800 2150 50  0001 C CNN
F 1 "GND" H 11805 2227 50  0000 C CNN
F 2 "" H 11800 2400 50  0001 C CNN
F 3 "" H 11800 2400 50  0001 C CNN
	1    11800 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	11800 2400 11800 2350
$Comp
L power:+3V3 #PWR016
U 1 1 5DBDD44B
P 11800 1600
F 0 "#PWR016" H 11800 1450 50  0001 C CNN
F 1 "+3V3" H 11815 1773 50  0000 C CNN
F 2 "" H 11800 1600 50  0001 C CNN
F 3 "" H 11800 1600 50  0001 C CNN
	1    11800 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	11800 1600 11800 1650
$Comp
L Device:R R6
U 1 1 5DBF6744
P 14600 3000
F 0 "R6" V 14393 3000 50  0000 C CNN
F 1 "180" V 14484 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 14530 3000 50  0001 C CNN
F 3 "~" H 14600 3000 50  0001 C CNN
	1    14600 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	15000 3000 15000 2100
Wire Wire Line
	14750 3000 15000 3000
Wire Wire Line
	14600 1750 14600 1800
$Comp
L power:GND #PWR023
U 1 1 5DBF6753
P 14600 2450
F 0 "#PWR023" H 14600 2200 50  0001 C CNN
F 1 "GND" H 14605 2277 50  0000 C CNN
F 2 "" H 14600 2450 50  0001 C CNN
F 3 "" H 14600 2450 50  0001 C CNN
	1    14600 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	14600 2450 14600 2400
Wire Wire Line
	14450 3000 14300 3000
Wire Wire Line
	14300 3000 14300 2200
Wire Wire Line
	14300 2200 14400 2200
$Comp
L power:+1V8 #PWR025
U 1 1 5DBF675D
P 13750 2950
F 0 "#PWR025" H 13750 2800 50  0001 C CNN
F 1 "+1V8" H 13765 3123 50  0000 C CNN
F 2 "" H 13750 2950 50  0001 C CNN
F 3 "" H 13750 2950 50  0001 C CNN
	1    13750 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5DBF6763
P 14050 3000
F 0 "R5" V 13843 3000 50  0000 C CNN
F 1 "1k" V 13934 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 13980 3000 50  0001 C CNN
F 3 "~" H 14050 3000 50  0001 C CNN
	1    14050 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	13900 3000 13750 3000
Wire Wire Line
	13750 3000 13750 2950
Wire Wire Line
	14200 3000 14300 3000
Connection ~ 14300 3000
$Comp
L Device:R R2
U 1 1 5DBF6773
P 13750 2200
F 0 "R2" H 13820 2246 50  0000 L CNN
F 1 "1.8k" H 13820 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 13680 2200 50  0001 C CNN
F 3 "~" H 13750 2200 50  0001 C CNN
	1    13750 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 1950 13750 2000
Connection ~ 13750 2000
Wire Wire Line
	13750 2000 13750 2050
$Comp
L power:GND #PWR021
U 1 1 5DBF677D
P 13750 2400
F 0 "#PWR021" H 13750 2150 50  0001 C CNN
F 1 "GND" H 13755 2227 50  0000 C CNN
F 2 "" H 13750 2400 50  0001 C CNN
F 3 "" H 13750 2400 50  0001 C CNN
	1    13750 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 2400 13750 2350
$Comp
L power:+3V3 #PWR017
U 1 1 5DBF6784
P 13750 1600
F 0 "#PWR017" H 13750 1450 50  0001 C CNN
F 1 "+3V3" H 13765 1773 50  0000 C CNN
F 2 "" H 13750 1600 50  0001 C CNN
F 3 "" H 13750 1600 50  0001 C CNN
	1    13750 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 1600 13750 1650
$Comp
L Device:Thermistor_NTC TH2
U 1 1 5DBF7608
P 13750 1800
F 0 "TH2" H 13848 1846 50  0000 L CNN
F 1 "Thermistor_NTC" H 13848 1755 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00012_1x02_P5.00mm_Horizontal" H 13750 1850 50  0001 C CNN
F 3 "~" H 13750 1850 50  0001 C CNN
	1    13750 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 2000 14400 2000
Wire Wire Line
	11800 2000 12450 2000
Text Notes 11750 1300 0    100  ~ 0
Motor
Text Notes 13650 1300 0    100  ~ 0
IGBTs
Text Notes 12150 1000 0    100  ~ 20
Temperature Sensors
Text Label 13050 2100 0    50   ~ 0
Motor_temp
Text Label 15000 2100 0    50   ~ 0
Inverter_temp
$Comp
L Amplifier_Operational:OPA356xxDBV U4
U 1 1 5DC75D11
P 12750 2100
F 0 "U4" H 12800 2400 50  0000 L CNN
F 1 "OPA356xxDBV" H 12800 2300 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 12650 1900 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa356.pdf" H 12750 2300 50  0001 C CNN
	1    12750 2100
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:OPA356xxDBV U5
U 1 1 5DC78013
P 14700 2100
F 0 "U5" H 14750 2400 50  0000 L CNN
F 1 "OPA356xxDBV" H 14750 2300 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 14600 1900 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa356.pdf" H 14700 2300 50  0001 C CNN
	1    14700 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3400 1800 3400
Wire Wire Line
	1900 3400 1900 3500
Connection ~ 1900 3400
Wire Wire Line
	1800 3400 1800 3500
Wire Wire Line
	2000 3500 2000 3400
Wire Wire Line
	2000 3400 1900 3400
Wire Wire Line
	2100 3500 2100 3400
Wire Wire Line
	2100 3400 2000 3400
Connection ~ 2000 3400
Wire Wire Line
	2200 3500 2200 3400
Wire Wire Line
	2200 3400 2100 3400
Connection ~ 2100 3400
$Comp
L power:GND #PWR027
U 1 1 5DC7D7B5
P 1900 7200
F 0 "#PWR027" H 1900 6950 50  0001 C CNN
F 1 "GND" H 1905 7027 50  0000 C CNN
F 2 "" H 1900 7200 50  0001 C CNN
F 3 "" H 1900 7200 50  0001 C CNN
	1    1900 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 7200 1800 7200
Wire Wire Line
	1800 7200 1800 7100
Wire Wire Line
	1900 7100 1900 7200
Connection ~ 1900 7200
Wire Wire Line
	2000 7100 2000 7200
Wire Wire Line
	2000 7200 1900 7200
$Comp
L Device:C C12
U 1 1 5DC86F64
P 1300 1200
F 0 "C12" H 1415 1246 50  0000 L CNN
F 1 "2.2n" H 1415 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1338 1050 50  0001 C CNN
F 3 "~" H 1300 1200 50  0001 C CNN
	1    1300 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5DC85555
P 900 1200
F 0 "C11" H 1015 1246 50  0000 L CNN
F 1 "2.2n" H 1015 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 938 1050 50  0001 C CNN
F 3 "~" H 900 1200 50  0001 C CNN
	1    900  1200
	1    0    0    -1  
$EndComp
Text Label 1200 4100 2    50   ~ 0
VCAP_1
Text Label 1200 4200 2    50   ~ 0
VCAP_2
Text Label 900  1050 2    50   ~ 0
VCAP_1
Text Label 1300 1050 2    50   ~ 0
VCAP_2
$Comp
L power:GND #PWR028
U 1 1 5DC8FD4A
P 900 1400
F 0 "#PWR028" H 900 1150 50  0001 C CNN
F 1 "GND" H 905 1227 50  0000 C CNN
F 2 "" H 900 1400 50  0001 C CNN
F 3 "" H 900 1400 50  0001 C CNN
	1    900  1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  1400 900  1350
$Comp
L power:GND #PWR029
U 1 1 5DC90D40
P 1300 1400
F 0 "#PWR029" H 1300 1150 50  0001 C CNN
F 1 "GND" H 1305 1227 50  0000 C CNN
F 2 "" H 1300 1400 50  0001 C CNN
F 3 "" H 1300 1400 50  0001 C CNN
	1    1300 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1400 1300 1350
Text Notes 1850 700  0    100  ~ 20
Decoupling
$Comp
L Device:C C13
U 1 1 5DC9211F
P 1800 1200
F 0 "C13" H 1915 1246 50  0000 L CNN
F 1 "100n" H 1915 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1838 1050 50  0001 C CNN
F 3 "~" H 1800 1200 50  0001 C CNN
	1    1800 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5DC92662
P 2200 1200
F 0 "C14" H 2315 1246 50  0000 L CNN
F 1 "1u" H 2315 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2238 1050 50  0001 C CNN
F 3 "~" H 2200 1200 50  0001 C CNN
	1    2200 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 1000 1800 1050
Wire Wire Line
	2200 1000 2200 1050
Text Notes 1750 950  0    50   ~ 0
ADC
NoConn ~ 1700 3500
$Comp
L Device:C C15
U 1 1 5DC9A5FB
P 2650 1200
F 0 "C15" H 2765 1246 50  0000 L CNN
F 1 "100n" H 2765 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2688 1050 50  0001 C CNN
F 3 "~" H 2650 1200 50  0001 C CNN
	1    2650 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5DC9A601
P 3050 1200
F 0 "C16" H 3165 1246 50  0000 L CNN
F 1 "100n" H 3165 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3088 1050 50  0001 C CNN
F 3 "~" H 3050 1200 50  0001 C CNN
	1    3050 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1000 2650 1050
Wire Wire Line
	3050 1000 3050 1050
$Comp
L Device:C C17
U 1 1 5DC9D027
P 3450 1200
F 0 "C17" H 3565 1246 50  0000 L CNN
F 1 "100n" H 3565 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3488 1050 50  0001 C CNN
F 3 "~" H 3450 1200 50  0001 C CNN
	1    3450 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C18
U 1 1 5DC9D02D
P 3850 1200
F 0 "C18" H 3965 1246 50  0000 L CNN
F 1 "100n" H 3965 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3888 1050 50  0001 C CNN
F 3 "~" H 3850 1200 50  0001 C CNN
	1    3850 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5DC9D033
P 3250 1450
F 0 "#PWR031" H 3250 1200 50  0001 C CNN
F 1 "GND" H 3255 1277 50  0000 C CNN
F 2 "" H 3250 1450 50  0001 C CNN
F 3 "" H 3250 1450 50  0001 C CNN
	1    3250 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1000 3450 1050
Wire Wire Line
	3850 1000 3850 1050
Wire Wire Line
	2650 1000 3050 1000
Connection ~ 3050 1000
Wire Wire Line
	3850 1000 3450 1000
Connection ~ 3450 1000
$Comp
L power:GND #PWR030
U 1 1 5DCA5BD5
P 2000 1450
F 0 "#PWR030" H 2000 1200 50  0001 C CNN
F 1 "GND" H 2005 1277 50  0000 C CNN
F 2 "" H 2000 1450 50  0001 C CNN
F 3 "" H 2000 1450 50  0001 C CNN
	1    2000 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1450 1800 1450
Wire Wire Line
	1800 1450 1800 1350
Wire Wire Line
	2200 1450 2000 1450
Wire Wire Line
	2200 1350 2200 1450
Connection ~ 2000 1450
Wire Wire Line
	3250 1450 3050 1450
Wire Wire Line
	2650 1350 2650 1450
Wire Wire Line
	3050 1350 3050 1450
Connection ~ 3050 1450
Wire Wire Line
	3050 1450 2650 1450
Wire Wire Line
	3850 1350 3850 1450
Wire Wire Line
	3450 1350 3450 1450
Connection ~ 3450 1450
Wire Wire Line
	3450 1450 3850 1450
Wire Wire Line
	3250 1450 3450 1450
Connection ~ 3250 1450
$Comp
L Device:C C7
U 1 1 5DCD607F
P 14300 1000
F 0 "C7" H 14415 1046 50  0000 L CNN
F 1 "100n" H 14415 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 14338 850 50  0001 C CNN
F 3 "~" H 14300 1000 50  0001 C CNN
	1    14300 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5DCD611B
P 14700 1000
F 0 "C8" H 14815 1046 50  0000 L CNN
F 1 "100n" H 14815 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 14738 850 50  0001 C CNN
F 3 "~" H 14700 1000 50  0001 C CNN
	1    14700 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	14300 800  14300 850 
Wire Wire Line
	14700 800  14700 850 
$Comp
L power:GND #PWR019
U 1 1 5DCDBE01
P 14500 1200
F 0 "#PWR019" H 14500 950 50  0001 C CNN
F 1 "GND" H 14505 1027 50  0000 C CNN
F 2 "" H 14500 1200 50  0001 C CNN
F 3 "" H 14500 1200 50  0001 C CNN
	1    14500 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	14500 1200 14300 1200
Wire Wire Line
	14300 1200 14300 1150
Wire Wire Line
	14700 1150 14700 1200
Wire Wire Line
	14700 1200 14500 1200
Connection ~ 14500 1200
$Comp
L Device:C C9
U 1 1 5DCE7793
P 15150 1000
F 0 "C9" H 15265 1046 50  0000 L CNN
F 1 "100n" H 15265 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 15188 850 50  0001 C CNN
F 3 "~" H 15150 1000 50  0001 C CNN
	1    15150 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5DCE7799
P 15550 1000
F 0 "C10" H 15665 1046 50  0000 L CNN
F 1 "100n" H 15665 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 15588 850 50  0001 C CNN
F 3 "~" H 15550 1000 50  0001 C CNN
	1    15550 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	15150 800  15150 850 
Wire Wire Line
	15550 800  15550 850 
$Comp
L power:GND #PWR026
U 1 1 5DCE77A4
P 15350 1200
F 0 "#PWR026" H 15350 950 50  0001 C CNN
F 1 "GND" H 15355 1027 50  0000 C CNN
F 2 "" H 15350 1200 50  0001 C CNN
F 3 "" H 15350 1200 50  0001 C CNN
	1    15350 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	15350 1200 15150 1200
Wire Wire Line
	15150 1200 15150 1150
Wire Wire Line
	15550 1150 15550 1200
Wire Wire Line
	15550 1200 15350 1200
Connection ~ 15350 1200
Wire Wire Line
	15150 800  15350 800 
$Comp
L power:+1V8 #PWR06
U 1 1 5DCE986F
P 15350 800
F 0 "#PWR06" H 15350 650 50  0001 C CNN
F 1 "+1V8" H 15365 973 50  0000 C CNN
F 2 "" H 15350 800 50  0001 C CNN
F 3 "" H 15350 800 50  0001 C CNN
	1    15350 800 
	1    0    0    -1  
$EndComp
Connection ~ 15350 800 
Wire Wire Line
	15350 800  15550 800 
Wire Wire Line
	14300 800  14500 800 
$Comp
L power:+3V3 #PWR01
U 1 1 5DCEA29A
P 14500 800
F 0 "#PWR01" H 14500 650 50  0001 C CNN
F 1 "+3V3" H 14515 973 50  0000 C CNN
F 2 "" H 14500 800 50  0001 C CNN
F 3 "" H 14500 800 50  0001 C CNN
	1    14500 800 
	1    0    0    -1  
$EndComp
Connection ~ 14500 800 
Wire Wire Line
	14500 800  14700 800 
$Comp
L power:+3V3 #PWR032
U 1 1 5DCEA848
P 12650 1750
F 0 "#PWR032" H 12650 1600 50  0001 C CNN
F 1 "+3V3" H 12665 1923 50  0000 C CNN
F 2 "" H 12650 1750 50  0001 C CNN
F 3 "" H 12650 1750 50  0001 C CNN
	1    12650 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR033
U 1 1 5DCEABBE
P 14600 1750
F 0 "#PWR033" H 14600 1600 50  0001 C CNN
F 1 "+3V3" H 14615 1923 50  0000 C CNN
F 2 "" H 14600 1750 50  0001 C CNN
F 3 "" H 14600 1750 50  0001 C CNN
	1    14600 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR034
U 1 1 5DCEBEA8
P 2000 3400
F 0 "#PWR034" H 2000 3250 50  0001 C CNN
F 1 "+3V3" H 2015 3573 50  0000 C CNN
F 2 "" H 2000 3400 50  0001 C CNN
F 3 "" H 2000 3400 50  0001 C CNN
	1    2000 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 1000 2000 1000
$Comp
L power:+3V3 #PWR011
U 1 1 5DCEC182
P 2000 1000
F 0 "#PWR011" H 2000 850 50  0001 C CNN
F 1 "+3V3" H 2015 1173 50  0000 C CNN
F 2 "" H 2000 1000 50  0001 C CNN
F 3 "" H 2000 1000 50  0001 C CNN
	1    2000 1000
	1    0    0    -1  
$EndComp
Connection ~ 2000 1000
Wire Wire Line
	2000 1000 2200 1000
$Comp
L power:+3V3 #PWR018
U 1 1 5DCEC636
P 3450 1000
F 0 "#PWR018" H 3450 850 50  0001 C CNN
F 1 "+3V3" H 3465 1173 50  0000 C CNN
F 2 "" H 3450 1000 50  0001 C CNN
F 3 "" H 3450 1000 50  0001 C CNN
	1    3450 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 5DD27FA0
P 4250 1200
F 0 "C19" H 4365 1246 50  0000 L CNN
F 1 "4.7u" H 4365 1155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4288 1050 50  0001 C CNN
F 3 "~" H 4250 1200 50  0001 C CNN
	1    4250 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1050 4250 1000
Wire Wire Line
	4250 1000 3850 1000
Connection ~ 3850 1000
Wire Wire Line
	4250 1350 4250 1450
Wire Wire Line
	4250 1450 3850 1450
Connection ~ 3850 1450
Wire Wire Line
	3050 1000 3450 1000
$Comp
L Connector:Conn_01x10_Female J1
U 1 1 5DD2DFA2
P 13400 10550
F 0 "J1" H 13428 10526 50  0000 L CNN
F 1 "Conn_01x10_Female" H 13428 10435 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x10_P2.54mm_Vertical" H 13400 10550 50  0001 C CNN
F 3 "~" H 13400 10550 50  0001 C CNN
	1    13400 10550
	1    0    0    -1  
$EndComp
Text Notes 13050 9800 0    100  ~ 20
IMU
$Comp
L power:+3V3 #PWR038
U 1 1 5DD331B4
P 13100 10100
F 0 "#PWR038" H 13100 9950 50  0001 C CNN
F 1 "+3V3" H 13115 10273 50  0000 C CNN
F 2 "" H 13100 10100 50  0001 C CNN
F 3 "" H 13100 10100 50  0001 C CNN
	1    13100 10100
	1    0    0    -1  
$EndComp
Wire Wire Line
	13100 10100 13100 10150
Wire Wire Line
	13100 10150 13200 10150
$Comp
L power:GND #PWR039
U 1 1 5DD3B561
P 12500 10550
F 0 "#PWR039" H 12500 10300 50  0001 C CNN
F 1 "GND" H 12505 10377 50  0000 C CNN
F 2 "" H 12500 10550 50  0001 C CNN
F 3 "" H 12500 10550 50  0001 C CNN
	1    12500 10550
	1    0    0    -1  
$EndComp
Text Label 13200 10350 2    50   ~ 0
SPI1_SCK
Text Label 13200 10450 2    50   ~ 0
SPI1_MOSI
Text Label 13150 10700 2    50   ~ 0
SPI1_MISO
Wire Wire Line
	13150 10700 13200 10700
Wire Wire Line
	13200 10700 13200 10650
Wire Wire Line
	13200 10750 13200 10700
Connection ~ 13200 10700
Text Label 13200 10850 2    50   ~ 0
SPI1_CS_Accel
Text Label 13200 10950 2    50   ~ 0
SPI1_CS_Gyro
Text Label 13200 11050 2    50   ~ 0
SPI1_CS_Magnet
Wire Wire Line
	12500 10550 13200 10550
Wire Wire Line
	12500 10550 12500 10250
Wire Wire Line
	12500 10250 13200 10250
Connection ~ 12500 10550
$Comp
L power:GND #PWR041
U 1 1 5DD53313
P 800 3900
F 0 "#PWR041" H 800 3650 50  0001 C CNN
F 1 "GND" H 805 3727 50  0000 C CNN
F 2 "" H 800 3900 50  0001 C CNN
F 3 "" H 800 3900 50  0001 C CNN
	1    800  3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  3900 1200 3900
Text Label 1050 3700 2    50   ~ 0
reset
$Comp
L power:+3V3 #PWR040
U 1 1 5DD5B29D
P 1100 3250
F 0 "#PWR040" H 1100 3100 50  0001 C CNN
F 1 "+3V3" H 1115 3423 50  0000 C CNN
F 2 "" H 1100 3250 50  0001 C CNN
F 3 "" H 1100 3250 50  0001 C CNN
	1    1100 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5DD5BE82
P 1100 3450
F 0 "R7" H 1170 3496 50  0000 L CNN
F 1 "10k" H 1170 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1030 3450 50  0001 C CNN
F 3 "~" H 1100 3450 50  0001 C CNN
	1    1100 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 3250 1100 3300
Wire Wire Line
	1050 3700 1100 3700
Wire Wire Line
	1100 3600 1100 3700
Connection ~ 1100 3700
Wire Wire Line
	1100 3700 1200 3700
Text Label 2600 4200 0    50   ~ 0
SPI1_SCK
Text Label 2600 4400 0    50   ~ 0
SPI1_MOSI
$Comp
L Device:C C20
U 1 1 5DC95CFF
P 950 2500
F 0 "C20" H 1065 2546 50  0000 L CNN
F 1 "12p" H 1065 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 988 2350 50  0001 C CNN
F 3 "~" H 950 2500 50  0001 C CNN
	1    950  2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C21
U 1 1 5DC96103
P 1350 2500
F 0 "C21" H 1465 2546 50  0000 L CNN
F 1 "12p" H 1465 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1388 2350 50  0001 C CNN
F 3 "~" H 1350 2500 50  0001 C CNN
	1    1350 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 5DC96831
P 1150 2250
F 0 "Y1" H 1150 2518 50  0000 C CNN
F 1 "32.768MHz" H 1150 2427 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm" H 1150 2250 50  0001 C CNN
F 3 "~" H 1150 2250 50  0001 C CNN
	1    1150 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 2250 950  2250
Wire Wire Line
	950  2250 950  2350
Wire Wire Line
	1300 2250 1350 2250
$Comp
L power:GND #PWR042
U 1 1 5DC9E145
P 1150 2700
F 0 "#PWR042" H 1150 2450 50  0001 C CNN
F 1 "GND" H 1155 2527 50  0000 C CNN
F 2 "" H 1150 2700 50  0001 C CNN
F 3 "" H 1150 2700 50  0001 C CNN
	1    1150 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 2700 950  2700
Wire Wire Line
	950  2700 950  2650
Wire Wire Line
	1350 2650 1350 2700
Wire Wire Line
	1350 2700 1150 2700
Connection ~ 1150 2700
Text Notes 1400 1900 0    100  ~ 20
Crystals
$Comp
L Device:C C22
U 1 1 5DCAD277
P 2150 2500
F 0 "C22" H 2265 2546 50  0000 L CNN
F 1 "30p" H 2265 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2188 2350 50  0001 C CNN
F 3 "~" H 2150 2500 50  0001 C CNN
	1    2150 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C23
U 1 1 5DCAD27D
P 2900 2500
F 0 "C23" H 3015 2546 50  0000 L CNN
F 1 "30p" H 3015 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2938 2350 50  0001 C CNN
F 3 "~" H 2900 2500 50  0001 C CNN
	1    2900 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 2250 2150 2250
Wire Wire Line
	2150 2250 2150 2350
Wire Wire Line
	2700 2250 2900 2250
Wire Wire Line
	2900 2250 2900 2350
$Comp
L power:GND #PWR043
U 1 1 5DCAD28D
P 2550 2700
F 0 "#PWR043" H 2550 2450 50  0001 C CNN
F 1 "GND" H 2555 2527 50  0000 C CNN
F 2 "" H 2550 2700 50  0001 C CNN
F 3 "" H 2550 2700 50  0001 C CNN
	1    2550 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2700 2150 2700
Wire Wire Line
	2150 2700 2150 2650
Wire Wire Line
	2900 2650 2900 2700
Wire Wire Line
	2900 2700 2550 2700
Connection ~ 2550 2700
Text Label 2600 4300 0    50   ~ 0
SPI1_MISO
Text Label 2600 4100 0    50   ~ 0
SPI1_CS_Accel
Text Label 1200 5800 2    50   ~ 0
SPI1_CS_Gyro
Text Label 1200 5900 2    50   ~ 0
SPI1_CS_Magnet
Text Label 2600 5400 0    50   ~ 0
Motor_temp
Text Label 2600 5500 0    50   ~ 0
Inverter_temp
NoConn ~ 10400 -1700
Text Label 15150 8400 0    50   ~ 0
Current_PH1
Text Label 15150 9000 0    50   ~ 0
Current_PH2
Text Label 15150 9600 0    50   ~ 0
Current_PH3
Text Label 2600 3700 0    50   ~ 0
Current_PH1
Text Label 2600 3800 0    50   ~ 0
Current_PH2
Text Label 2600 3900 0    50   ~ 0
Current_PH3
Text Label 1350 2250 0    50   ~ 0
LSE_OUT
Text Label 950  2250 2    50   ~ 0
LSE_IN
Text Label 2150 2250 2    50   ~ 0
HSE_IN
Wire Wire Line
	2550 2050 2550 2000
Wire Wire Line
	2550 2000 2800 2000
$Comp
L power:GND #PWR044
U 1 1 5DCCC2B5
P 2800 2000
F 0 "#PWR044" H 2800 1750 50  0001 C CNN
F 1 "GND" H 2805 1827 50  0000 C CNN
F 2 "" H 2800 2000 50  0001 C CNN
F 3 "" H 2800 2000 50  0001 C CNN
	1    2800 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal_GND24 Y2
U 1 1 5DCB2832
P 2550 2250
F 0 "Y2" H 2150 2450 50  0000 L CNN
F 1 "25MHz" H 2150 2350 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 2550 2250 50  0001 C CNN
F 3 "~" H 2550 2250 50  0001 C CNN
	1    2550 2250
	1    0    0    -1  
$EndComp
Text Label 2900 2250 0    50   ~ 0
HSE_OUT
Text Label 1200 6900 2    50   ~ 0
LSE_OUT
Text Label 1200 6800 2    50   ~ 0
LSE_IN
Text Label 1200 4900 2    50   ~ 0
HSE_IN
Text Label 1200 5000 2    50   ~ 0
HSE_OUT
Text Label 2600 6700 0    50   ~ 0
SPI2_SCK
Text Label 2600 6900 0    50   ~ 0
SPI2_MOSI
Text Label 2600 6800 0    50   ~ 0
SPI2_MISO
Text Label 2600 6600 0    50   ~ 0
SPI2_NSS
Text Label 1200 6000 2    50   ~ 0
FOC_Status
$Comp
L Interface_CAN_LIN:MCP2562-E-SN U8
U 1 1 5DD9E2BC
P 2000 9000
F 0 "U8" H 2200 9500 50  0000 C CNN
F 1 "MCP2562-E-SN" H 2450 9400 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2000 8500 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/25167A.pdf" H 2000 9000 50  0001 C CNN
	1    2000 9000
	1    0    0    -1  
$EndComp
Text Notes 1500 8200 0    100  ~ 20
CAN Interface
$Comp
L power:+5V #PWR050
U 1 1 5DDB6224
P 2000 8550
F 0 "#PWR050" H 2000 8400 50  0001 C CNN
F 1 "+5V" H 2015 8723 50  0000 C CNN
F 2 "" H 2000 8550 50  0001 C CNN
F 3 "" H 2000 8550 50  0001 C CNN
	1    2000 8550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 8550 2000 8600
$Comp
L power:+3V3 #PWR051
U 1 1 5DDBD09A
P 1050 9100
F 0 "#PWR051" H 1050 8950 50  0001 C CNN
F 1 "+3V3" H 1065 9273 50  0000 C CNN
F 2 "" H 1050 9100 50  0001 C CNN
F 3 "" H 1050 9100 50  0001 C CNN
	1    1050 9100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 9100 1500 9100
$Comp
L power:GND #PWR053
U 1 1 5DDC4767
P 2000 9400
F 0 "#PWR053" H 2000 9150 50  0001 C CNN
F 1 "GND" H 2005 9227 50  0000 C CNN
F 2 "" H 2000 9400 50  0001 C CNN
F 3 "" H 2000 9400 50  0001 C CNN
	1    2000 9400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J3
U 1 1 5DDC5D1F
P 2800 9050
F 0 "J3" H 2772 8932 50  0000 R CNN
F 1 "Conn_01x02_Male" H 2772 9023 50  0000 R CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBVA_2,5_2-G-5,08_1x02_P5.08mm_Vertical" H 2800 9050 50  0001 C CNN
F 3 "~" H 2800 9050 50  0001 C CNN
	1    2800 9050
	-1   0    0    1   
$EndComp
Wire Wire Line
	2600 8950 2600 8900
Wire Wire Line
	2600 9050 2600 9100
Wire Wire Line
	2600 9100 2500 9100
Wire Wire Line
	2600 8900 2500 8900
Text Label 1500 8800 2    50   ~ 0
CAN_RX
Text Label 1500 8900 2    50   ~ 0
CAN_TX
Text Label 2600 4800 0    50   ~ 0
CAN_RX
Text Label 2600 4900 0    50   ~ 0
CAN_TX
Text Label 1500 9200 2    50   ~ 0
CAN_STBY
Text Label 2600 4700 0    50   ~ 0
CAN_STBY
$Comp
L fiat:TMC4671-ES U1
U 1 1 5DCA1D60
P 5650 4650
F 0 "U1" H 6400 2650 40  0000 C CNN
F 1 "TMC4671-ES" H 6550 2550 40  0000 C CNN
F 2 "trinamic:TMC4671-ES" H 6700 4850 60  0001 L CNN
F 3 "https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC4671_datasheet_v1.03.pdf" H 6700 4950 60  0001 L CNN
F 4 "1460-1302-ND" H 6700 5050 60  0001 L CNN "Digi-Key_PN"
F 5 "TMC4671-ES" H 6700 5150 60  0001 L CNN "MPN"
F 6 "Integrated Circuits (ICs)" H 6700 5250 60  0001 L CNN "Category"
F 7 "PMIC - Motor Drivers, Controllers" H 6700 5350 60  0001 L CNN "Family"
F 8 "https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC4671_datasheet_v1.03.pdf" H 6700 5450 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/trinamic-motion-control-gmbh/TMC4671-ES/1460-1302-ND/9450492" H 6700 5550 60  0001 L CNN "DK_Detail_Page"
F 10 "IC MOTOR DRIVER MULTIPHASE 76QFN" H 6700 5650 60  0001 L CNN "Description"
F 11 "Trinamic Motion Control GmbH" H 6700 5750 60  0001 L CNN "Manufacturer"
F 12 "Active" H 6700 5850 60  0001 L CNN "Status"
	1    5650 4650
	1    0    0    -1  
$EndComp
NoConn ~ 6850 4750
NoConn ~ 6850 4850
NoConn ~ 6850 4950
NoConn ~ 6850 5050
NoConn ~ 6850 5150
NoConn ~ 6850 5250
NoConn ~ 6850 5550
$Comp
L power:GND #PWR052
U 1 1 5DCE5884
P 5700 6750
F 0 "#PWR052" H 5700 6500 50  0001 C CNN
F 1 "GND" H 5705 6577 50  0000 C CNN
F 2 "" H 5700 6750 50  0001 C CNN
F 3 "" H 5700 6750 50  0001 C CNN
	1    5700 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 6750 5650 6750
Wire Wire Line
	5350 6750 5350 6650
Wire Wire Line
	5450 6650 5450 6750
Connection ~ 5450 6750
Wire Wire Line
	5450 6750 5350 6750
Wire Wire Line
	5550 6650 5550 6750
Connection ~ 5550 6750
Wire Wire Line
	5550 6750 5450 6750
Wire Wire Line
	5650 6650 5650 6750
Connection ~ 5650 6750
Wire Wire Line
	5650 6750 5550 6750
Wire Wire Line
	6050 6650 6050 6750
Wire Wire Line
	6050 6750 5950 6750
Connection ~ 5700 6750
Wire Wire Line
	5750 6650 5750 6750
Connection ~ 5750 6750
Wire Wire Line
	5750 6750 5700 6750
Wire Wire Line
	5850 6650 5850 6750
Connection ~ 5850 6750
Wire Wire Line
	5850 6750 5750 6750
Wire Wire Line
	5950 6650 5950 6750
Connection ~ 5950 6750
Wire Wire Line
	5950 6750 5850 6750
Text Label 4550 6150 2    50   ~ 0
SPI2_SCK
Text Label 4550 6350 2    50   ~ 0
SPI2_MOSI
Text Label 4550 6450 2    50   ~ 0
SPI2_MISO
Text Label 4550 6250 2    50   ~ 0
SPI2_NSS
NoConn ~ 6850 4350
NoConn ~ 6850 4450
NoConn ~ 6850 4550
$Comp
L Connector_Generic:Conn_02x13_Odd_Even J2
U 1 1 5DD38A7B
P 8700 4600
F 0 "J2" H 8750 5417 50  0000 C CNN
F 1 "Conn_02x13_Odd_Even" H 8750 5326 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x13_P2.54mm_Vertical_Lock" H 8700 4600 50  0001 C CNN
F 3 "~" H 8700 4600 50  0001 C CNN
	1    8700 4600
	1    0    0    -1  
$EndComp
Text Notes 8150 3600 0    100  ~ 20
IGBT Connector
Text Notes 5150 2400 0    100  ~ 20
FOC Controller
$Comp
L power:GND #PWR049
U 1 1 5DD523D2
P 8450 5250
F 0 "#PWR049" H 8450 5000 50  0001 C CNN
F 1 "GND" H 8455 5077 50  0000 C CNN
F 2 "" H 8450 5250 50  0001 C CNN
F 3 "" H 8450 5250 50  0001 C CNN
	1    8450 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 5250 8450 5200
Wire Wire Line
	8450 4900 8500 4900
Wire Wire Line
	8500 5000 8450 5000
Connection ~ 8450 5000
Wire Wire Line
	8450 5000 8450 4900
Wire Wire Line
	8500 5100 8450 5100
Connection ~ 8450 5100
Wire Wire Line
	8450 5100 8450 5000
Wire Wire Line
	8500 5200 8450 5200
Connection ~ 8450 5200
Wire Wire Line
	8450 5200 8450 5100
$Comp
L power:+24V #PWR046
U 1 1 5DD6A898
P 9300 4550
F 0 "#PWR046" H 9300 4400 50  0001 C CNN
F 1 "+24V" H 9315 4723 50  0000 C CNN
F 2 "" H 9300 4550 50  0001 C CNN
F 3 "" H 9300 4550 50  0001 C CNN
	1    9300 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 4550 9300 4600
Wire Wire Line
	9300 4600 9000 4600
$Comp
L power:+24V #PWR047
U 1 1 5DD7161B
P 8150 4650
F 0 "#PWR047" H 8150 4500 50  0001 C CNN
F 1 "+24V" H 8165 4823 50  0000 C CNN
F 2 "" H 8150 4650 50  0001 C CNN
F 3 "" H 8150 4650 50  0001 C CNN
	1    8150 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4650 8150 4700
Wire Wire Line
	8150 4700 8500 4700
$Comp
L power:GND #PWR048
U 1 1 5DD8F9C1
P 9400 4800
F 0 "#PWR048" H 9400 4550 50  0001 C CNN
F 1 "GND" H 9405 4627 50  0000 C CNN
F 2 "" H 9400 4800 50  0001 C CNN
F 3 "" H 9400 4800 50  0001 C CNN
	1    9400 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 4800 9000 4800
Text Label 4550 4250 2    50   ~ 0
FOC_Status
$Comp
L MCU_ST_STM32F4:STM32F405RGTx U6
U 1 1 5DC72DB0
P 1900 5300
F 0 "U6" H 2200 3400 50  0000 C CNN
F 1 "STM32F405RGTx" H 2450 3500 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 1300 3600 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00037051.pdf" H 1900 5300 50  0001 C CNN
	1    1900 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2350 1350 2250
Wire Wire Line
	2550 2450 2550 2700
$Comp
L Device:R R9
U 1 1 5DCEF592
P 10800 4250
F 0 "R9" H 10870 4296 50  0000 L CNN
F 1 "300k" H 10870 4205 50  0000 L CNN
F 2 "" V 10730 4250 50  0001 C CNN
F 3 "~" H 10800 4250 50  0001 C CNN
	1    10800 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR057
U 1 1 5DCF04F2
P 11550 5000
F 0 "#PWR057" H 11550 4750 50  0001 C CNN
F 1 "GND" H 11555 4827 50  0000 C CNN
F 2 "" H 11550 5000 50  0001 C CNN
F 3 "" H 11550 5000 50  0001 C CNN
	1    11550 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C28
U 1 1 5DCF0BE0
P 14250 5050
F 0 "C28" H 14365 5096 50  0000 L CNN
F 1 "1n" H 14365 5005 50  0000 L CNN
F 2 "" H 14288 4900 50  0001 C CNN
F 3 "~" H 14250 5050 50  0001 C CNN
	1    14250 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5DCF3C49
P 10800 4750
F 0 "R12" H 10870 4796 50  0000 L CNN
F 1 "2k" H 10870 4705 50  0000 L CNN
F 2 "" V 10730 4750 50  0001 C CNN
F 3 "~" H 10800 4750 50  0001 C CNN
	1    10800 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5DCF818A
P 11300 4500
F 0 "R10" V 11093 4500 50  0000 C CNN
F 1 "39" V 11184 4500 50  0000 C CNN
F 2 "" V 11230 4500 50  0001 C CNN
F 3 "~" H 11300 4500 50  0001 C CNN
	1    11300 4500
	0    1    1    0   
$EndComp
$Comp
L Device:C C25
U 1 1 5DCFA772
P 11550 4750
F 0 "C25" H 11665 4796 50  0000 L CNN
F 1 "10n" H 11665 4705 50  0000 L CNN
F 2 "" H 11588 4600 50  0001 C CNN
F 3 "~" H 11550 4750 50  0001 C CNN
	1    11550 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5DD01CA7
P 14650 5100
F 0 "R14" H 14720 5146 50  0000 L CNN
F 1 "20k" H 14720 5055 50  0000 L CNN
F 2 "" V 14580 5100 50  0001 C CNN
F 3 "~" H 14650 5100 50  0001 C CNN
	1    14650 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C26
U 1 1 5DD02351
P 12050 4750
F 0 "C26" H 12165 4796 50  0000 L CNN
F 1 "0.1u" H 12165 4705 50  0000 L CNN
F 2 "" H 12088 4600 50  0001 C CNN
F 3 "~" H 12050 4750 50  0001 C CNN
	1    12050 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 3900 10800 4100
Wire Wire Line
	10800 4400 10800 4500
Wire Wire Line
	10800 4900 10800 5000
Wire Wire Line
	10800 5000 11550 5000
Wire Wire Line
	11550 4900 11550 5000
Connection ~ 11550 5000
Wire Wire Line
	12050 4900 12050 5000
Wire Wire Line
	12050 5000 11550 5000
Wire Wire Line
	11150 4500 10800 4500
Connection ~ 10800 4500
Wire Wire Line
	10800 4500 10800 4600
Wire Wire Line
	11450 4500 11550 4500
Wire Wire Line
	11550 4500 11550 4600
Connection ~ 11550 4500
Wire Wire Line
	12550 4500 11550 4500
$Comp
L Isolator_Analog:ACPL-C87A U9
U 1 1 5DCED4E3
P 12950 4700
F 0 "U9" H 12550 5250 50  0000 L CNN
F 1 "ACPL-C87A" H 12550 5150 50  0000 L CNN
F 2 "Package_SO:SSO-8_6.8x5.9mm_P1.27mm_Clearance8mm" H 13100 4350 50  0001 L CIN
F 3 "www.avagotech.com/docs/AV02-3563EN" H 12995 4705 50  0001 L CNN
	1    12950 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	12850 4300 12050 4300
Wire Wire Line
	12050 4300 12050 4600
$Comp
L power:GND #PWR059
U 1 1 5DD174F3
P 12550 5150
F 0 "#PWR059" H 12550 4900 50  0001 C CNN
F 1 "GND" H 12555 4977 50  0000 C CNN
F 2 "" H 12550 5150 50  0001 C CNN
F 3 "" H 12550 5150 50  0001 C CNN
	1    12550 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	12550 4700 12550 5100
Wire Wire Line
	12850 5100 12550 5100
Connection ~ 12550 5100
Wire Wire Line
	12550 5100 12550 5150
$Comp
L Device:C C27
U 1 1 5DD1D61E
P 13600 5050
F 0 "C27" H 13715 5096 50  0000 L CNN
F 1 "0.1u" H 13715 5005 50  0000 L CNN
F 2 "" H 13638 4900 50  0001 C CNN
F 3 "~" H 13600 5050 50  0001 C CNN
	1    13600 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR060
U 1 1 5DD1DFAF
P 13600 5350
F 0 "#PWR060" H 13600 5100 50  0001 C CNN
F 1 "GND" H 13605 5177 50  0000 C CNN
F 2 "" H 13600 5350 50  0001 C CNN
F 3 "" H 13600 5350 50  0001 C CNN
	1    13600 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	13050 4300 13050 4200
Wire Wire Line
	13050 4200 13600 4200
Wire Wire Line
	13600 5200 13600 5300
Wire Wire Line
	13050 5100 13050 5300
Wire Wire Line
	13050 5300 13600 5300
Connection ~ 13600 5300
Wire Wire Line
	13600 5300 13600 5350
$Comp
L Device:R R11
U 1 1 5DD229E3
P 13950 4600
F 0 "R11" V 13743 4600 50  0000 C CNN
F 1 "220" V 13834 4600 50  0000 C CNN
F 2 "" V 13880 4600 50  0001 C CNN
F 3 "~" H 13950 4600 50  0001 C CNN
	1    13950 4600
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 5DD238DA
P 13950 4800
F 0 "R13" V 13743 4800 50  0000 C CNN
F 1 "220" V 13834 4800 50  0000 C CNN
F 2 "" V 13880 4800 50  0001 C CNN
F 3 "~" H 13950 4800 50  0001 C CNN
	1    13950 4800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	13350 4800 13800 4800
Wire Wire Line
	13350 4600 13800 4600
Wire Wire Line
	14100 4800 14250 4800
Wire Wire Line
	13600 5300 14250 5300
Wire Wire Line
	14650 5250 14650 5300
Wire Wire Line
	14250 5200 14250 5300
Connection ~ 14250 5300
Wire Wire Line
	14250 5300 14650 5300
Wire Wire Line
	14250 4900 14250 4800
Connection ~ 14250 4800
Wire Wire Line
	14250 4800 14650 4800
Wire Wire Line
	14650 4950 14650 4800
Connection ~ 14650 4800
$Comp
L Device:C C24
U 1 1 5DD39AB8
P 15100 4050
F 0 "C24" V 15352 4050 50  0000 C CNN
F 1 "1n" V 15261 4050 50  0000 C CNN
F 2 "" H 15138 3900 50  0001 C CNN
F 3 "~" H 15100 4050 50  0001 C CNN
	1    15100 4050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 5DD39ABE
P 15100 3700
F 0 "R8" V 15307 3700 50  0000 C CNN
F 1 "20k" V 15216 3700 50  0000 C CNN
F 2 "" V 15030 3700 50  0001 C CNN
F 3 "~" H 15100 3700 50  0001 C CNN
	1    15100 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	14950 4050 14650 4050
Wire Wire Line
	15250 4050 15500 4050
Wire Wire Line
	14950 3700 14650 3700
Wire Wire Line
	14650 3700 14650 4050
Connection ~ 14650 4050
Wire Wire Line
	15250 3700 15500 3700
Wire Wire Line
	15500 3700 15500 4050
$Comp
L Amplifier_Operational:OPA356xxDBV U10
U 1 1 5DF3AEA9
P 15200 4700
F 0 "U10" H 15300 4600 50  0000 L CNN
F 1 "OPA356xxDBV" H 15300 4500 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 15100 4500 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa356.pdf" H 15200 4900 50  0001 C CNN
	1    15200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	14650 4050 14650 4800
Wire Wire Line
	14650 4800 14900 4800
Wire Wire Line
	14100 4600 14900 4600
$Comp
L power:+3V3 #PWR056
U 1 1 5DF5F562
P 15100 4400
F 0 "#PWR056" H 15100 4250 50  0001 C CNN
F 1 "+3V3" H 15115 4573 50  0000 C CNN
F 2 "" H 15100 4400 50  0001 C CNN
F 3 "" H 15100 4400 50  0001 C CNN
	1    15100 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR058
U 1 1 5DF6041B
P 15100 5000
F 0 "#PWR058" H 15100 4750 50  0001 C CNN
F 1 "GND" H 15105 4827 50  0000 C CNN
F 2 "" H 15100 5000 50  0001 C CNN
F 3 "" H 15100 5000 50  0001 C CNN
	1    15100 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	13600 4900 13600 4200
$Comp
L power:+3V3 #PWR054
U 1 1 5DFEAFF1
P 13600 4200
F 0 "#PWR054" H 13600 4050 50  0001 C CNN
F 1 "+3V3" H 13615 4373 50  0000 C CNN
F 2 "" H 13600 4200 50  0001 C CNN
F 3 "" H 13600 4200 50  0001 C CNN
	1    13600 4200
	1    0    0    -1  
$EndComp
Connection ~ 13600 4200
$Comp
L power:+3V3 #PWR055
U 1 1 5DFEB6A4
P 12050 4300
F 0 "#PWR055" H 12050 4150 50  0001 C CNN
F 1 "+3V3" H 12065 4473 50  0000 C CNN
F 2 "" H 12050 4300 50  0001 C CNN
F 3 "" H 12050 4300 50  0001 C CNN
	1    12050 4300
	1    0    0    -1  
$EndComp
Connection ~ 12050 4300
$Comp
L power:+BATT #PWR045
U 1 1 5DFEC8B7
P 10800 3900
F 0 "#PWR045" H 10800 3750 50  0001 C CNN
F 1 "+BATT" H 10815 4073 50  0000 C CNN
F 2 "" H 10800 3900 50  0001 C CNN
F 3 "" H 10800 3900 50  0001 C CNN
	1    10800 3900
	1    0    0    -1  
$EndComp
Text Notes 12250 3700 0    100  ~ 20
DC-Link Voltage Sensor
Connection ~ 15500 4050
Wire Wire Line
	15500 4050 15500 4700
Text Notes 13850 6000 0    100  ~ 20
AC Phase Voltage Sensors
$Sheet
S 14350 6700 700  200 
U 5E0181EF
F0 "AC Voltage Sensor Phase 2" 50
F1 "ac_sensor.sch" 50
F2 "out" O R 15050 6800 50 
F3 "ac_phase" I L 14350 6800 50 
$EndSheet
$Sheet
S 14350 7100 700  200 
U 5E0225B4
F0 "AC Voltage Sensor Phase 3" 50
F1 "ac_sensor.sch" 50
F2 "out" O R 15050 7200 50 
F3 "ac_phase" I L 14350 7200 50 
$EndSheet
$Sheet
S 14350 6300 700  200 
U 5DFF18CA
F0 "AC Voltage Sensor Phase 1" 50
F1 "ac_sensor.sch" 50
F2 "out" O R 15050 6400 50 
F3 "ac_phase" I L 14350 6400 50 
$EndSheet
$EndSCHEMATC
