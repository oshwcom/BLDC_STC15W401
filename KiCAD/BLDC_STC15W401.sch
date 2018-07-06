EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:STCMCU
LIBS:BLDC_STC15W401-cache
EELAYER 25 0
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
L R R4
U 1 1 5B2AF43D
P 1900 800
F 0 "R4" V 2000 800 50  0000 C CNN
F 1 "470" V 1800 800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1830 800 50  0001 C CNN
F 3 "" H 1900 800 50  0001 C CNN
	1    1900 800 
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 5B2AF94E
P 1100 1350
F 0 "R1" V 1200 1350 50  0000 C CNN
F 1 "1k" V 1000 1350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1030 1350 50  0001 C CNN
F 3 "" H 1100 1350 50  0001 C CNN
	1    1100 1350
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 5B2AF9B3
P 800 2900
F 0 "C1" H 825 3000 50  0000 L CNN
F 1 "104" H 825 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 838 2750 50  0001 C CNN
F 3 "" H 800 2900 50  0001 C CNN
	1    800  2900
	1    0    0    -1  
$EndComp
$Comp
L DTC143Z Q1
U 1 1 5B2AFA53
P 1600 1350
F 0 "Q1" H 1800 1425 50  0000 L CNN
F 1 "DTC143Z" H 1800 1350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 1600 1350 50  0001 L CNN
F 3 "" H 1600 1350 50  0001 L CNN
	1    1600 1350
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR01
U 1 1 5B2AFB2A
P 2150 750
F 0 "#PWR01" H 2150 600 50  0001 C CNN
F 1 "+12V" H 2150 890 50  0000 C CNN
F 2 "" H 2150 750 50  0001 C CNN
F 3 "" H 2150 750 50  0001 C CNN
	1    2150 750 
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q3
U 1 1 5B2B0ADA
P 2050 1850
F 0 "Q3" H 2250 1900 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 2250 1800 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 2250 1950 50  0001 C CNN
F 3 "" H 2050 1850 50  0001 C CNN
	1    2050 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5B2B0BBC
P 2150 2200
F 0 "#PWR02" H 2150 1950 50  0001 C CNN
F 1 "GND" H 2150 2050 50  0000 C CNN
F 2 "" H 2150 2200 50  0001 C CNN
F 3 "" H 2150 2200 50  0001 C CNN
	1    2150 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5B2B0C77
P 1700 1600
F 0 "#PWR03" H 1700 1350 50  0001 C CNN
F 1 "GND" H 1700 1450 50  0000 C CNN
F 2 "" H 1700 1600 50  0001 C CNN
F 3 "" H 1700 1600 50  0001 C CNN
	1    1700 1600
	1    0    0    -1  
$EndComp
Text GLabel 800  1350 0    60   Input ~ 0
A+
$Comp
L R R2
U 1 1 5B2B0DCF
P 1100 1850
F 0 "R2" V 1200 1850 50  0000 C CNN
F 1 "100" V 1000 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1030 1850 50  0001 C CNN
F 3 "" H 1100 1850 50  0001 C CNN
	1    1100 1850
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 5B2B0E19
P 1650 2150
F 0 "R3" V 1750 2150 50  0000 C CNN
F 1 "1K" V 1550 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1580 2150 50  0001 C CNN
F 3 "" H 1650 2150 50  0001 C CNN
	1    1650 2150
	0    1    1    0   
$EndComp
Text GLabel 800  1850 0    60   Input ~ 0
A-
Text GLabel 2300 1450 2    60   Input ~ 0
A
Wire Wire Line
	1700 1050 1850 1050
Wire Wire Line
	1700 800  1700 1150
Connection ~ 1700 1050
Wire Wire Line
	2150 750  2150 850 
Wire Wire Line
	2150 1250 2150 1650
Wire Wire Line
	2150 2050 2150 2200
Wire Wire Line
	2150 800  2050 800 
Connection ~ 2150 800 
Wire Wire Line
	1700 1550 1700 1600
Wire Wire Line
	800  1350 950  1350
Wire Wire Line
	1800 2150 2150 2150
Connection ~ 2150 2150
Wire Wire Line
	1250 1850 1850 1850
Wire Wire Line
	1350 2150 1500 2150
Wire Wire Line
	1350 1850 1350 2150
Connection ~ 1350 1850
Wire Wire Line
	950  1850 800  1850
Wire Wire Line
	2150 1450 2300 1450
Connection ~ 2150 1450
Wire Wire Line
	1250 1350 1350 1350
Wire Wire Line
	1750 800  1700 800 
Wire Notes Line
	550  2400 5100 2400
$Comp
L R R11
U 1 1 5B2B23B3
P 4200 800
F 0 "R11" V 4300 800 50  0000 C CNN
F 1 "470" V 4100 800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4130 800 50  0001 C CNN
F 3 "" H 4200 800 50  0001 C CNN
	1    4200 800 
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 5B2B23B9
P 3400 1350
F 0 "R7" V 3500 1350 50  0000 C CNN
F 1 "1k" V 3300 1350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3330 1350 50  0001 C CNN
F 3 "" H 3400 1350 50  0001 C CNN
	1    3400 1350
	0    1    1    0   
$EndComp
$Comp
L DTC143Z Q4
U 1 1 5B2B23BF
P 3900 1350
F 0 "Q4" H 4100 1425 50  0000 L CNN
F 1 "DTC143Z" H 4100 1350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 3900 1350 50  0001 L CNN
F 3 "" H 3900 1350 50  0001 L CNN
	1    3900 1350
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR04
U 1 1 5B2B23C5
P 4450 750
F 0 "#PWR04" H 4450 600 50  0001 C CNN
F 1 "+12V" H 4450 890 50  0000 C CNN
F 2 "" H 4450 750 50  0001 C CNN
F 3 "" H 4450 750 50  0001 C CNN
	1    4450 750 
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q6
U 1 1 5B2B23D1
P 4350 1850
F 0 "Q6" H 4550 1900 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 4550 1800 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 4550 1950 50  0001 C CNN
F 3 "" H 4350 1850 50  0001 C CNN
	1    4350 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5B2B23D7
P 4450 2200
F 0 "#PWR05" H 4450 1950 50  0001 C CNN
F 1 "GND" H 4450 2050 50  0000 C CNN
F 2 "" H 4450 2200 50  0001 C CNN
F 3 "" H 4450 2200 50  0001 C CNN
	1    4450 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5B2B23DD
P 4000 1600
F 0 "#PWR06" H 4000 1350 50  0001 C CNN
F 1 "GND" H 4000 1450 50  0000 C CNN
F 2 "" H 4000 1600 50  0001 C CNN
F 3 "" H 4000 1600 50  0001 C CNN
	1    4000 1600
	1    0    0    -1  
$EndComp
Text GLabel 3100 1350 0    60   Input ~ 0
B+
$Comp
L R R8
U 1 1 5B2B23E4
P 3400 1850
F 0 "R8" V 3500 1850 50  0000 C CNN
F 1 "100" V 3300 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3330 1850 50  0001 C CNN
F 3 "" H 3400 1850 50  0001 C CNN
	1    3400 1850
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 5B2B23EA
P 3950 2150
F 0 "R10" V 4050 2150 50  0000 C CNN
F 1 "1K" V 3850 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3880 2150 50  0001 C CNN
F 3 "" H 3950 2150 50  0001 C CNN
	1    3950 2150
	0    1    1    0   
$EndComp
Text GLabel 3100 1850 0    60   Input ~ 0
B-
Text GLabel 4600 1450 2    60   Input ~ 0
B
Wire Wire Line
	4000 1050 4150 1050
Wire Wire Line
	4000 800  4000 1150
Connection ~ 4000 1050
Wire Wire Line
	4450 750  4450 850 
Wire Wire Line
	4450 1250 4450 1650
Wire Wire Line
	4450 2050 4450 2200
Wire Wire Line
	4450 800  4350 800 
Connection ~ 4450 800 
Wire Wire Line
	4000 1550 4000 1600
Wire Wire Line
	3100 1350 3250 1350
Wire Wire Line
	4100 2150 4450 2150
Connection ~ 4450 2150
Wire Wire Line
	3550 1850 4150 1850
Wire Wire Line
	3650 2150 3800 2150
Wire Wire Line
	3650 1750 3650 2150
Connection ~ 3650 1850
Wire Wire Line
	3250 1850 3100 1850
Wire Wire Line
	4450 1450 4600 1450
Connection ~ 4450 1450
Wire Wire Line
	3550 1350 3650 1350
Wire Wire Line
	4050 800  4000 800 
Wire Notes Line
	2850 2400 7450 2400
$Comp
L R R16
U 1 1 5B2B25E7
P 6550 800
F 0 "R16" V 6650 800 50  0000 C CNN
F 1 "470" V 6450 800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6480 800 50  0001 C CNN
F 3 "" H 6550 800 50  0001 C CNN
	1    6550 800 
	0    1    1    0   
$EndComp
$Comp
L R R12
U 1 1 5B2B25ED
P 5750 1350
F 0 "R12" V 5850 1350 50  0000 C CNN
F 1 "1k" V 5650 1350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5680 1350 50  0001 C CNN
F 3 "" H 5750 1350 50  0001 C CNN
	1    5750 1350
	0    1    1    0   
$EndComp
$Comp
L DTC143Z Q7
U 1 1 5B2B25F3
P 6250 1350
F 0 "Q7" H 6450 1425 50  0000 L CNN
F 1 "DTC143Z" H 6450 1350 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 6250 1350 50  0001 L CNN
F 3 "" H 6250 1350 50  0001 L CNN
	1    6250 1350
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR07
U 1 1 5B2B25F9
P 6800 750
F 0 "#PWR07" H 6800 600 50  0001 C CNN
F 1 "+12V" H 6800 890 50  0000 C CNN
F 2 "" H 6800 750 50  0001 C CNN
F 3 "" H 6800 750 50  0001 C CNN
	1    6800 750 
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q9
U 1 1 5B2B2605
P 6700 1850
F 0 "Q9" H 6900 1900 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 6900 1800 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 6900 1950 50  0001 C CNN
F 3 "" H 6700 1850 50  0001 C CNN
	1    6700 1850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5B2B260B
P 6800 2200
F 0 "#PWR08" H 6800 1950 50  0001 C CNN
F 1 "GND" H 6800 2050 50  0000 C CNN
F 2 "" H 6800 2200 50  0001 C CNN
F 3 "" H 6800 2200 50  0001 C CNN
	1    6800 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5B2B2611
P 6350 1600
F 0 "#PWR09" H 6350 1350 50  0001 C CNN
F 1 "GND" H 6350 1450 50  0000 C CNN
F 2 "" H 6350 1600 50  0001 C CNN
F 3 "" H 6350 1600 50  0001 C CNN
	1    6350 1600
	1    0    0    -1  
$EndComp
Text GLabel 5450 1350 0    60   Input ~ 0
C+
$Comp
L R R13
U 1 1 5B2B2618
P 5750 1850
F 0 "R13" V 5850 1850 50  0000 C CNN
F 1 "100" V 5650 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5680 1850 50  0001 C CNN
F 3 "" H 5750 1850 50  0001 C CNN
	1    5750 1850
	0    1    1    0   
$EndComp
$Comp
L R R14
U 1 1 5B2B261E
P 6300 2150
F 0 "R14" V 6400 2150 50  0000 C CNN
F 1 "1K" V 6200 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6230 2150 50  0001 C CNN
F 3 "" H 6300 2150 50  0001 C CNN
	1    6300 2150
	0    1    1    0   
$EndComp
Text GLabel 5450 1850 0    60   Input ~ 0
C-
Text GLabel 6950 1450 2    60   Input ~ 0
C
Wire Wire Line
	6350 1050 6500 1050
Wire Wire Line
	6350 800  6350 1150
Connection ~ 6350 1050
Wire Wire Line
	6800 750  6800 850 
Wire Wire Line
	6800 1250 6800 1650
Wire Wire Line
	6800 2050 6800 2200
Wire Wire Line
	6800 800  6700 800 
Connection ~ 6800 800 
Wire Wire Line
	6350 1550 6350 1600
Wire Wire Line
	5450 1350 5600 1350
Wire Wire Line
	6450 2150 6800 2150
Connection ~ 6800 2150
Wire Wire Line
	5900 1850 6500 1850
Wire Wire Line
	6000 2150 6150 2150
Wire Wire Line
	6000 1800 6000 2150
Connection ~ 6000 1850
Wire Wire Line
	5600 1850 5450 1850
Wire Wire Line
	6800 1450 6950 1450
Connection ~ 6800 1450
Wire Wire Line
	5900 1350 6000 1350
Wire Wire Line
	6400 800  6350 800 
Wire Notes Line
	7450 2400 7450 550 
$Comp
L L78L05_SOT89 U1
U 1 1 5B2B29E8
P 1550 2750
F 0 "U1" H 1400 2875 50  0000 C CNN
F 1 "L78L05_SOT89" H 1525 2875 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-89-3_Handsoldering" H 1550 2950 50  0001 C CIN
F 3 "" H 1550 2700 50  0001 C CNN
	1    1550 2750
	1    0    0    -1  
$EndComp
$Comp
L CP C2
U 1 1 5B2B2D25
P 1100 2900
F 0 "C2" H 1125 3000 50  0000 L CNN
F 1 "470uF" H 1125 2800 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D6.3mm_P2.50mm" H 1138 2750 50  0001 C CNN
F 3 "" H 1100 2900 50  0001 C CNN
	1    1100 2900
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5B2B31D3
P 2300 2900
F 0 "C4" H 2325 3000 50  0000 L CNN
F 1 "104" H 2325 2800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2338 2750 50  0001 C CNN
F 3 "" H 2300 2900 50  0001 C CNN
	1    2300 2900
	1    0    0    -1  
$EndComp
$Comp
L CP C3
U 1 1 5B2B3321
P 2000 2900
F 0 "C3" H 2025 3000 50  0000 L CNN
F 1 "220uF" H 2025 2800 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.00mm" H 2038 2750 50  0001 C CNN
F 3 "" H 2000 2900 50  0001 C CNN
	1    2000 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  2750 1250 2750
Connection ~ 1100 2750
Wire Wire Line
	800  3050 2300 3050
Connection ~ 1100 3050
Connection ~ 1550 3050
Connection ~ 2000 3050
Wire Wire Line
	1850 2750 2300 2750
Connection ~ 2000 2750
$Comp
L +12V #PWR010
U 1 1 5B2B3BE0
P 800 2750
F 0 "#PWR010" H 800 2600 50  0001 C CNN
F 1 "+12V" H 800 2890 50  0000 C CNN
F 2 "" H 800 2750 50  0001 C CNN
F 3 "" H 800 2750 50  0001 C CNN
	1    800  2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 5B2B3C2A
P 800 3050
F 0 "#PWR011" H 800 2800 50  0001 C CNN
F 1 "GND" H 800 2900 50  0000 C CNN
F 2 "" H 800 3050 50  0001 C CNN
F 3 "" H 800 3050 50  0001 C CNN
	1    800  3050
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR012
U 1 1 5B2B3CAA
P 2300 2750
F 0 "#PWR012" H 2300 2600 50  0001 C CNN
F 1 "+5V" H 2300 2890 50  0000 C CNN
F 2 "" H 2300 2750 50  0001 C CNN
F 3 "" H 2300 2750 50  0001 C CNN
	1    2300 2750
	1    0    0    -1  
$EndComp
Wire Notes Line
	550  3350 2600 3350
Wire Notes Line
	2600 3350 2600 2450
Wire Notes Line
	2600 2450 550  2450
Wire Notes Line
	550  2450 550  3350
Text Notes 1450 3300 0    60   ~ 0
Power
Wire Notes Line
	550  2400 550  550 
Wire Notes Line
	550  550  7450 550 
$Comp
L STC15W401AS-SOP16 U2
U 1 1 5B2B6A3C
P 4900 3650
F 0 "U2" H 4900 4150 60  0000 C CNN
F 1 "STC15W401AS-SOP16" H 4900 3150 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 4800 3550 60  0001 C CNN
F 3 "" H 4800 3550 60  0001 C CNN
	1    4900 3650
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5B2B6FEC
P 9400 1800
F 0 "C7" H 9425 1900 50  0000 L CNN
F 1 "104" H 9425 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9438 1650 50  0001 C CNN
F 3 "" H 9400 1800 50  0001 C CNN
	1    9400 1800
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 5B2B725C
P 9600 1800
F 0 "C8" H 9625 1900 50  0000 L CNN
F 1 "104" H 9625 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9638 1650 50  0001 C CNN
F 3 "" H 9600 1800 50  0001 C CNN
	1    9600 1800
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 5B2B72CD
P 9800 1800
F 0 "C9" H 9825 1900 50  0000 L CNN
F 1 "104" H 9825 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9838 1650 50  0001 C CNN
F 3 "" H 9800 1800 50  0001 C CNN
	1    9800 1800
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J3
U 1 1 5B2B7457
P 10800 1200
F 0 "J3" H 10800 1400 50  0000 C CNN
F 1 "BLDC" H 10800 1000 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_B03B-XH-A_03x2.50mm_Straight" H 10800 1200 50  0001 C CNN
F 3 "" H 10800 1200 50  0001 C CNN
	1    10800 1200
	1    0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 5B2B7B10
P 8250 1500
F 0 "R19" V 8350 1500 50  0000 C CNN
F 1 "10k" V 8150 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8180 1500 50  0001 C CNN
F 3 "" H 8250 1500 50  0001 C CNN
	1    8250 1500
	0    1    1    0   
$EndComp
$Comp
L R R18
U 1 1 5B2B7D08
P 8250 1200
F 0 "R18" V 8350 1200 50  0000 C CNN
F 1 "10k" V 8150 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8180 1200 50  0001 C CNN
F 3 "" H 8250 1200 50  0001 C CNN
	1    8250 1200
	0    1    1    0   
$EndComp
$Comp
L R R17
U 1 1 5B2B7D85
P 8250 900
F 0 "R17" V 8350 900 50  0000 C CNN
F 1 "10k" V 8150 900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8180 900 50  0001 C CNN
F 3 "" H 8250 900 50  0001 C CNN
	1    8250 900 
	0    1    1    0   
$EndComp
$Comp
L R R25
U 1 1 5B2B8163
P 10250 1500
F 0 "R25" V 10350 1500 50  0000 C CNN
F 1 "10k" V 10150 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10180 1500 50  0001 C CNN
F 3 "" H 10250 1500 50  0001 C CNN
	1    10250 1500
	0    1    1    0   
$EndComp
$Comp
L R R24
U 1 1 5B2B8169
P 10250 1200
F 0 "R24" V 10350 1200 50  0000 C CNN
F 1 "10k" V 10150 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10180 1200 50  0001 C CNN
F 3 "" H 10250 1200 50  0001 C CNN
	1    10250 1200
	0    1    1    0   
$EndComp
$Comp
L R R23
U 1 1 5B2B816F
P 10250 900
F 0 "R23" V 10350 900 50  0000 C CNN
F 1 "10k" V 10150 900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10180 900 50  0001 C CNN
F 3 "" H 10250 900 50  0001 C CNN
	1    10250 900 
	0    1    1    0   
$EndComp
$Comp
L R R21
U 1 1 5B2B82A1
P 8850 1800
F 0 "R21" V 8950 1800 50  0000 C CNN
F 1 "2k" V 8750 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8780 1800 50  0001 C CNN
F 3 "" H 8850 1800 50  0001 C CNN
	1    8850 1800
	-1   0    0    1   
$EndComp
$Comp
L R R22
U 1 1 5B2B85FC
P 9150 1800
F 0 "R22" V 9250 1800 50  0000 C CNN
F 1 "2k" V 9050 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9080 1800 50  0001 C CNN
F 3 "" H 9150 1800 50  0001 C CNN
	1    9150 1800
	-1   0    0    1   
$EndComp
$Comp
L R R20
U 1 1 5B2B8CD5
P 8550 1800
F 0 "R20" V 8650 1800 50  0000 C CNN
F 1 "2k" V 8450 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8480 1800 50  0001 C CNN
F 3 "" H 8550 1800 50  0001 C CNN
	1    8550 1800
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 3400 4000 3400
Text GLabel 4000 3400 0    47   Input ~ 0
P13
Wire Wire Line
	4150 3500 4000 3500
Text GLabel 4000 3500 0    47   Input ~ 0
P14
Wire Wire Line
	4150 3600 4000 3600
Text GLabel 4000 3600 0    47   Input ~ 0
P15
Wire Wire Line
	4150 3700 4000 3700
Text GLabel 4000 3700 0    47   Input ~ 0
P54
Wire Wire Line
	8100 900  8000 900 
Wire Wire Line
	8000 900  8000 1500
Wire Wire Line
	7900 1200 8100 1200
Wire Wire Line
	8000 1500 8100 1500
Connection ~ 8000 1200
Text GLabel 7900 1200 0    60   Input ~ 0
P54
Text GLabel 10000 850  1    47   Input ~ 0
P13
Text GLabel 10000 1150 1    47   Input ~ 0
P14
Text GLabel 10000 1450 1    47   Input ~ 0
P15
Wire Wire Line
	8400 900  10100 900 
Wire Wire Line
	8400 1500 10100 1500
Text GLabel 10450 1450 1    60   Input ~ 0
C
Text GLabel 10450 850  1    60   Input ~ 0
A
Text GLabel 10450 1150 1    60   Input ~ 0
B
Wire Wire Line
	8400 1200 10100 1200
Wire Wire Line
	10000 1450 10000 1500
Connection ~ 10000 1500
Wire Wire Line
	10000 1150 10000 1200
Connection ~ 10000 1200
Wire Wire Line
	10000 850  10000 900 
Connection ~ 10000 900 
Wire Wire Line
	10400 900  10600 900 
Wire Wire Line
	10600 900  10600 1100
Wire Wire Line
	10450 850  10450 900 
Connection ~ 10450 900 
Wire Wire Line
	10400 1200 10600 1200
Wire Wire Line
	10450 1150 10450 1200
Connection ~ 10450 1200
Wire Wire Line
	10600 1500 10600 1300
Wire Wire Line
	10400 1500 10600 1500
Wire Wire Line
	10450 1450 10450 1500
Connection ~ 10450 1500
Wire Wire Line
	8550 900  8550 1650
Connection ~ 8550 900 
Wire Wire Line
	8850 1200 8850 1650
Connection ~ 8850 1200
Wire Wire Line
	9150 1500 9150 1650
Connection ~ 9150 1500
Wire Wire Line
	9400 900  9400 1650
Connection ~ 9400 900 
Wire Wire Line
	9600 1200 9600 1650
Connection ~ 9600 1200
Wire Wire Line
	9800 1500 9800 1650
Connection ~ 9800 1500
Wire Wire Line
	8550 1950 8550 2050
Wire Wire Line
	8550 2050 9800 2050
Wire Wire Line
	9800 2050 9800 1950
Wire Wire Line
	8850 1950 8850 2050
Connection ~ 8850 2050
Wire Wire Line
	9150 1950 9150 2150
Connection ~ 9150 2050
Wire Wire Line
	9400 1950 9400 2050
Connection ~ 9400 2050
Wire Wire Line
	9600 1950 9600 2050
Connection ~ 9600 2050
$Comp
L GND #PWR013
U 1 1 5B2BF9C3
P 9150 2150
F 0 "#PWR013" H 9150 1900 50  0001 C CNN
F 1 "GND" H 9150 2000 50  0000 C CNN
F 2 "" H 9150 2150 50  0001 C CNN
F 3 "" H 9150 2150 50  0001 C CNN
	1    9150 2150
	1    0    0    -1  
$EndComp
Wire Notes Line
	7550 550  7550 2400
Wire Notes Line
	7550 2400 11100 2400
Wire Notes Line
	11100 2400 11100 550 
Wire Notes Line
	11100 550  7550 550 
Wire Wire Line
	5650 3900 5800 3900
Wire Wire Line
	5650 4000 5800 4000
Text GLabel 5800 4000 2    47   Input ~ 0
RX
Text GLabel 5800 3900 2    47   Input ~ 0
TX
$Comp
L Conn_01x04 J1
U 1 1 5B2C2DC8
P 850 4000
F 0 "J1" H 850 4200 50  0000 C CNN
F 1 "Download" H 850 3700 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_B04B-XH-A_04x2.50mm_Straight" H 850 4000 50  0001 C CNN
F 3 "" H 850 4000 50  0001 C CNN
	1    850  4000
	-1   0    0    1   
$EndComp
Wire Wire Line
	1050 3800 1300 3800
Wire Wire Line
	1050 3900 1300 3900
Wire Wire Line
	1050 4100 1300 4100
Wire Wire Line
	1050 4000 1600 4000
$Comp
L +5V #PWR014
U 1 1 5B2C33C2
P 1300 3800
F 0 "#PWR014" H 1300 3650 50  0001 C CNN
F 1 "+5V" H 1300 3940 50  0000 C CNN
F 2 "" H 1300 3800 50  0001 C CNN
F 3 "" H 1300 3800 50  0001 C CNN
	1    1300 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 5B2C377C
P 1600 4000
F 0 "#PWR015" H 1600 3750 50  0001 C CNN
F 1 "GND" H 1600 3850 50  0000 C CNN
F 2 "" H 1600 4000 50  0001 C CNN
F 3 "" H 1600 4000 50  0001 C CNN
	1    1600 4000
	1    0    0    -1  
$EndComp
Text GLabel 1300 3900 2    47   Input ~ 0
TX
Text GLabel 1300 4100 2    47   Input ~ 0
RX
$Comp
L LED D1
U 1 1 5B2C4719
P 1950 3800
F 0 "D1" H 1950 3900 50  0000 C CNN
F 1 "LED" H 1950 3700 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 1950 3800 50  0001 C CNN
F 3 "" H 1950 3800 50  0001 C CNN
	1    1950 3800
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 5B2C5557
P 1950 4150
F 0 "R5" V 2050 4150 50  0000 C CNN
F 1 "1k" V 1850 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1880 4150 50  0001 C CNN
F 3 "" H 1950 4150 50  0001 C CNN
	1    1950 4150
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR016
U 1 1 5B2C68E7
P 1950 3600
F 0 "#PWR016" H 1950 3450 50  0001 C CNN
F 1 "+5V" H 1950 3740 50  0000 C CNN
F 2 "" H 1950 3600 50  0001 C CNN
F 3 "" H 1950 3600 50  0001 C CNN
	1    1950 3600
	1    0    0    -1  
$EndComp
Text GLabel 1950 4400 3    47   Input ~ 0
TX
Wire Wire Line
	1950 3600 1950 3650
Wire Wire Line
	1950 3950 1950 4000
Wire Wire Line
	1950 4300 1950 4400
$Comp
L LED D2
U 1 1 5B2C6F9D
P 2300 3800
F 0 "D2" H 2300 3900 50  0000 C CNN
F 1 "LED" H 2300 3700 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 2300 3800 50  0001 C CNN
F 3 "" H 2300 3800 50  0001 C CNN
	1    2300 3800
	0    -1   -1   0   
$EndComp
$Comp
L R R6
U 1 1 5B2C6FA3
P 2300 4150
F 0 "R6" V 2400 4150 50  0000 C CNN
F 1 "1k" V 2200 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2230 4150 50  0001 C CNN
F 3 "" H 2300 4150 50  0001 C CNN
	1    2300 4150
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR017
U 1 1 5B2C6FA9
P 2300 3600
F 0 "#PWR017" H 2300 3450 50  0001 C CNN
F 1 "+5V" H 2300 3740 50  0000 C CNN
F 2 "" H 2300 3600 50  0001 C CNN
F 3 "" H 2300 3600 50  0001 C CNN
	1    2300 3600
	1    0    0    -1  
$EndComp
Text GLabel 2300 4400 3    47   Input ~ 0
RX
Wire Wire Line
	2300 3600 2300 3650
Wire Wire Line
	2300 3950 2300 4000
Wire Wire Line
	2300 4300 2300 4400
Wire Notes Line
	550  3400 550  4600
Wire Notes Line
	550  4600 2600 4600
Wire Notes Line
	2600 4600 2600 3400
Wire Notes Line
	2600 3400 550  3400
Text Notes 1250 4500 0    60   ~ 0
Download
Wire Wire Line
	5650 3800 6150 3800
$Comp
L R R15
U 1 1 5B2C815D
P 6300 3800
F 0 "R15" V 6400 3800 50  0000 C CNN
F 1 "470" V 6200 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6230 3800 50  0001 C CNN
F 3 "" H 6300 3800 50  0001 C CNN
	1    6300 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	6450 3800 6800 3800
$Comp
L Conn_01x02 J2
U 1 1 5B2C8533
P 7000 3800
F 0 "J2" H 7000 3900 50  0000 C CNN
F 1 "PPM" H 7000 3600 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 7000 3800 50  0001 C CNN
F 3 "" H 7000 3800 50  0001 C CNN
	1    7000 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3900 6650 3900
Wire Wire Line
	6650 3900 6650 4150
$Comp
L GND #PWR018
U 1 1 5B2C87D6
P 6650 4150
F 0 "#PWR018" H 6650 3900 50  0001 C CNN
F 1 "GND" H 6650 4000 50  0000 C CNN
F 2 "" H 6650 4150 50  0001 C CNN
F 3 "" H 6650 4150 50  0001 C CNN
	1    6650 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3800 4150 3800
Wire Wire Line
	4150 4000 3650 4000
$Comp
L C C6
U 1 1 5B2C8CCC
P 3400 4000
F 0 "C6" H 3425 4100 50  0000 L CNN
F 1 "104" H 3425 3900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3438 3850 50  0001 C CNN
F 3 "" H 3400 4000 50  0001 C CNN
	1    3400 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3750 3150 3850
Wire Wire Line
	3400 3850 3400 3800
Connection ~ 3400 3800
Wire Wire Line
	3650 4000 3650 4200
Wire Wire Line
	3650 4200 3150 4200
Wire Wire Line
	3150 4150 3150 4250
Wire Wire Line
	3400 4150 3400 4200
Connection ~ 3400 4200
$Comp
L +5V #PWR019
U 1 1 5B2C971D
P 3150 3750
F 0 "#PWR019" H 3150 3600 50  0001 C CNN
F 1 "+5V" H 3150 3890 50  0000 C CNN
F 2 "" H 3150 3750 50  0001 C CNN
F 3 "" H 3150 3750 50  0001 C CNN
	1    3150 3750
	1    0    0    -1  
$EndComp
Connection ~ 3150 3800
$Comp
L GND #PWR020
U 1 1 5B2C98C6
P 3150 4250
F 0 "#PWR020" H 3150 4000 50  0001 C CNN
F 1 "GND" H 3150 4100 50  0000 C CNN
F 2 "" H 3150 4250 50  0001 C CNN
F 3 "" H 3150 4250 50  0001 C CNN
	1    3150 4250
	1    0    0    -1  
$EndComp
Connection ~ 3150 4200
Text GLabel 5800 3300 2    47   Input ~ 0
A+
Text GLabel 5800 3600 2    47   Input ~ 0
A-
Text GLabel 5800 3400 2    47   Input ~ 0
B+
Text GLabel 5800 3700 2    47   Input ~ 0
B-
Text GLabel 5800 3500 2    47   Input ~ 0
C+
Text GLabel 4000 3900 0    47   Input ~ 0
C-
Wire Wire Line
	4000 3900 4150 3900
Wire Wire Line
	5650 3300 5800 3300
Wire Wire Line
	5650 3400 5800 3400
Wire Wire Line
	5650 3500 5800 3500
Wire Wire Line
	5650 3600 5800 3600
Wire Wire Line
	5650 3700 5800 3700
Wire Wire Line
	4000 3300 4150 3300
$Comp
L POT RV1
U 1 1 5B2CCD4E
P 3300 3050
F 0 "RV1" V 3125 3050 50  0000 C CNN
F 1 "RK09L 10K" V 3200 3050 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Alps_RK09L_Sleve_Single_Horizontal" H 3300 3050 50  0001 C CNN
F 3 "" H 3300 3050 50  0001 C CNN
	1    3300 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3050 3550 3050
Wire Wire Line
	3300 3300 3300 3200
Wire Wire Line
	3300 2900 3300 2800
Wire Wire Line
	4000 3300 4000 3050
$Comp
L +5V #PWR021
U 1 1 5B2CEDE2
P 3300 2800
F 0 "#PWR021" H 3300 2650 50  0001 C CNN
F 1 "+5V" H 3300 2940 50  0000 C CNN
F 2 "" H 3300 2800 50  0001 C CNN
F 3 "" H 3300 2800 50  0001 C CNN
	1    3300 2800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5B2CEEB9
P 3300 3300
F 0 "#PWR022" H 3300 3050 50  0001 C CNN
F 1 "GND" H 3300 3150 50  0000 C CNN
F 2 "" H 3300 3300 50  0001 C CNN
F 3 "" H 3300 3300 50  0001 C CNN
	1    3300 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3050 3850 3050
Wire Notes Line
	2700 2450 2700 4600
Wire Notes Line
	2700 4600 7450 4600
Wire Notes Line
	7450 4600 7450 2450
Wire Notes Line
	7450 2450 2700 2450
$Comp
L R R9
U 1 1 5B2CFC64
P 3700 3050
F 0 "R9" V 3800 3050 50  0000 C CNN
F 1 "470" V 3600 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3630 3050 50  0001 C CNN
F 3 "" H 3700 3050 50  0001 C CNN
	1    3700 3050
	0    1    1    0   
$EndComp
$Comp
L CP C5
U 1 1 5B2C904C
P 3150 4000
F 0 "C5" H 3175 4100 50  0000 L CNN
F 1 "47uF" H 3175 3900 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P1.50mm" H 3188 3850 50  0001 C CNN
F 3 "" H 3150 4000 50  0001 C CNN
	1    3150 4000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J4
U 1 1 5B2D4754
P 6900 2650
F 0 "J4" H 6900 2750 50  0000 C CNN
F 1 "DC12V" H 6900 2450 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 6900 2650 50  0001 C CNN
F 3 "" H 6900 2650 50  0001 C CNN
	1    6900 2650
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x02 J5
U 1 1 5B2D5143
P 6900 3150
F 0 "J5" H 6900 3250 50  0000 C CNN
F 1 "DC5V" H 6900 2950 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 6900 3150 50  0001 C CNN
F 3 "" H 6900 3150 50  0001 C CNN
	1    6900 3150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7100 2650 7200 2650
Wire Wire Line
	7100 2750 7200 2750
Wire Wire Line
	7100 3150 7200 3150
Wire Wire Line
	7100 3250 7200 3250
$Comp
L +5V #PWR023
U 1 1 5B2D56FF
P 7200 3150
F 0 "#PWR023" H 7200 3000 50  0001 C CNN
F 1 "+5V" H 7200 3290 50  0000 C CNN
F 2 "" H 7200 3150 50  0001 C CNN
F 3 "" H 7200 3150 50  0001 C CNN
	1    7200 3150
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR024
U 1 1 5B2D5B2E
P 7200 2650
F 0 "#PWR024" H 7200 2500 50  0001 C CNN
F 1 "+12V" H 7200 2790 50  0000 C CNN
F 2 "" H 7200 2650 50  0001 C CNN
F 3 "" H 7200 2650 50  0001 C CNN
	1    7200 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 5B2D6031
P 7200 2750
F 0 "#PWR025" H 7200 2500 50  0001 C CNN
F 1 "GND" H 7200 2600 50  0000 C CNN
F 2 "" H 7200 2750 50  0001 C CNN
F 3 "" H 7200 2750 50  0001 C CNN
	1    7200 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 5B2D624E
P 7200 3250
F 0 "#PWR026" H 7200 3000 50  0001 C CNN
F 1 "GND" H 7200 3100 50  0000 C CNN
F 2 "" H 7200 3250 50  0001 C CNN
F 3 "" H 7200 3250 50  0001 C CNN
	1    7200 3250
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J6
U 1 1 5B2D6CA1
P 6100 2950
F 0 "J6" H 6100 3250 50  0000 C CNN
F 1 "PWM" H 6100 2550 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_B06B-XH-A_06x2.50mm_Straight" H 6100 2950 50  0001 C CNN
F 3 "" H 6100 2950 50  0001 C CNN
	1    6100 2950
	-1   0    0    1   
$EndComp
Text GLabel 6450 2650 2    47   Input ~ 0
A+
Text GLabel 6450 2950 2    47   Input ~ 0
A-
Text GLabel 6450 2750 2    47   Input ~ 0
B+
Text GLabel 6450 3050 2    47   Input ~ 0
B-
Text GLabel 6450 2850 2    47   Input ~ 0
C+
Wire Wire Line
	6300 2650 6450 2650
Wire Wire Line
	6300 2750 6450 2750
Wire Wire Line
	6300 2850 6450 2850
Wire Wire Line
	6300 2950 6450 2950
Wire Wire Line
	6300 3050 6450 3050
Wire Wire Line
	6300 3150 6450 3150
Text GLabel 6450 3150 2    47   Input ~ 0
C-
Text Notes 2000 2750 0    47   ~ 0
100mA
Text Notes 850  2750 0    47   ~ 0
1.5A
$Comp
L Conn_01x03 J7
U 1 1 5B2DD248
P 10800 1850
F 0 "J7" H 10800 2050 50  0000 C CNN
F 1 "BLDC" H 10800 1650 50  0000 C CNN
F 2 "BLDC_STC15W401:JST_VH_B5P-VH-B_5A3x3.96mm_Vertical" H 10800 1850 50  0001 C CNN
F 3 "" H 10800 1850 50  0001 C CNN
	1    10800 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 1850 10450 1850
Wire Wire Line
	10600 1950 10500 1950
Text GLabel 10450 1700 0    60   Input ~ 0
A
Text GLabel 10450 1850 0    60   Input ~ 0
B
Text GLabel 10450 2000 0    60   Input ~ 0
C
Wire Wire Line
	10450 1700 10500 1700
Wire Wire Line
	10500 1700 10500 1750
Wire Wire Line
	10500 1750 10600 1750
Wire Wire Line
	10500 1950 10500 2000
Wire Wire Line
	10500 2000 10450 2000
$Comp
L Q_NMOS_GSD Q2
U 1 1 5B3EDB7E
P 2050 1050
F 0 "Q2" H 2250 1100 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 2250 1000 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 2250 1150 50  0001 C CNN
F 3 "" H 2050 1050 50  0001 C CNN
	1    2050 1050
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q5
U 1 1 5B3EDED3
P 4350 1050
F 0 "Q5" H 4550 1100 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 4550 1000 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 4550 1150 50  0001 C CNN
F 3 "" H 4350 1050 50  0001 C CNN
	1    4350 1050
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q8
U 1 1 5B3EE14F
P 6700 1050
F 0 "Q8" H 6900 1100 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 6900 1000 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 6900 1150 50  0001 C CNN
F 3 "" H 6700 1050 50  0001 C CNN
	1    6700 1050
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q10
U 1 1 5B3EFD04
P 8100 3100
F 0 "Q10" H 8300 3150 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 8300 3050 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8300 3200 50  0001 C CNN
F 3 "" H 8100 3100 50  0001 C CNN
	1    8100 3100
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q11
U 1 1 5B3F01F5
P 8100 4000
F 0 "Q11" H 8300 4050 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 8300 3950 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 8300 4100 50  0001 C CNN
F 3 "" H 8100 4000 50  0001 C CNN
	1    8100 4000
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q12
U 1 1 5B3F042A
P 9250 3100
F 0 "Q12" H 9450 3150 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 9450 3050 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 9450 3200 50  0001 C CNN
F 3 "" H 9250 3100 50  0001 C CNN
	1    9250 3100
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q13
U 1 1 5B3F04F2
P 9250 3950
F 0 "Q13" H 9450 4000 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 9450 3900 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 9450 4050 50  0001 C CNN
F 3 "" H 9250 3950 50  0001 C CNN
	1    9250 3950
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q14
U 1 1 5B3F0804
P 10550 3100
F 0 "Q14" H 10750 3150 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 10750 3050 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10750 3200 50  0001 C CNN
F 3 "" H 10550 3100 50  0001 C CNN
	1    10550 3100
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS Q15
U 1 1 5B3F08DA
P 10550 3950
F 0 "Q15" H 10750 4000 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 10750 3900 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 10750 4050 50  0001 C CNN
F 3 "" H 10550 3950 50  0001 C CNN
	1    10550 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 2900 8200 2700
Wire Wire Line
	8200 2700 10650 2700
Wire Wire Line
	9350 2650 9350 2900
Wire Wire Line
	10650 2700 10650 2900
Connection ~ 9350 2700
Wire Wire Line
	8200 3300 8200 3800
Wire Wire Line
	9350 3300 9350 3750
Wire Wire Line
	10650 3300 10650 3750
Wire Wire Line
	8200 4200 8200 4300
Wire Wire Line
	10650 4300 10650 4150
Wire Wire Line
	9350 4150 9350 4300
Wire Wire Line
	8200 4300 10650 4300
Connection ~ 9350 4300
$Comp
L GND #PWR027
U 1 1 5B3F2540
P 9350 4300
F 0 "#PWR027" H 9350 4050 50  0001 C CNN
F 1 "GND" H 9350 4150 50  0000 C CNN
F 2 "" H 9350 4300 50  0001 C CNN
F 3 "" H 9350 4300 50  0001 C CNN
	1    9350 4300
	1    0    0    -1  
$EndComp
Text GLabel 8400 3550 2    60   Input ~ 0
A
Text GLabel 9450 3550 2    60   Input ~ 0
B
Text GLabel 10750 3550 2    60   Input ~ 0
C
Text GLabel 1300 1300 1    60   Input ~ 0
A+G
Text GLabel 1350 1850 1    60   Input ~ 0
A-G
Text GLabel 3600 1200 1    60   Input ~ 0
B+G
Wire Wire Line
	1300 1300 1300 1350
Connection ~ 1300 1350
Wire Wire Line
	3600 1200 3600 1350
Connection ~ 3600 1350
Text GLabel 3650 1750 1    60   Input ~ 0
B-G
Text GLabel 5950 1300 1    60   Input ~ 0
C+G
Wire Wire Line
	5950 1300 5950 1350
Connection ~ 5950 1350
Text GLabel 6000 1800 1    60   Input ~ 0
C-G
Text GLabel 10300 3100 0    60   Input ~ 0
C+G
Wire Wire Line
	10300 3100 10350 3100
Wire Wire Line
	10750 3550 10650 3550
Connection ~ 10650 3550
Wire Wire Line
	9450 3550 9350 3550
Connection ~ 9350 3550
Wire Wire Line
	8400 3550 8200 3550
Connection ~ 8200 3550
Text Label 8000 6750 0    60   ~ 0
BLDC_STC15W408AS_V0.2_20180706
Text Label 8450 6950 0    60   ~ 0
QQ:25199527
Text Label 7650 7100 0    60   ~ 0
https://github.com/oshwcom/BLDC_STC15W401
Text GLabel 10300 3950 0    60   Input ~ 0
C-G
Wire Wire Line
	10300 3950 10350 3950
Text GLabel 8950 3100 0    60   Input ~ 0
B+G
Text GLabel 8950 3950 0    60   Input ~ 0
B-G
Text GLabel 7800 3100 0    60   Input ~ 0
A+G
Text GLabel 7800 4000 0    60   Input ~ 0
A-G
Wire Wire Line
	7800 4000 7900 4000
Wire Wire Line
	7800 3100 7900 3100
Wire Notes Line
	7550 2450 7550 4600
Wire Notes Line
	7550 4600 11100 4600
Wire Notes Line
	11100 4600 11100 2450
Wire Notes Line
	11100 2450 7550 2450
$Comp
L +12V #PWR028
U 1 1 5B3FC337
P 9350 2650
F 0 "#PWR028" H 9350 2500 50  0001 C CNN
F 1 "+12V" H 9350 2790 50  0000 C CNN
F 2 "" H 9350 2650 50  0001 C CNN
F 3 "" H 9350 2650 50  0001 C CNN
	1    9350 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3100 9050 3100
Wire Wire Line
	8950 3950 9050 3950
$EndSCHEMATC
