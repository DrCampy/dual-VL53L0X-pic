EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Frustrating Box - TOF Distance sensor"
Date "2019-12-04"
Rev "2.1"
Comp "Université de Liège"
Comment1 "Rev 1.1. Removing 3.3 / 5v jumpers.  Updating MCLR circuit to use shottky diode"
Comment2 "Rev 2.0 Replacing PIC16F by PIC24F16KL402"
Comment3 "Rev 2.1 Adding DIP, updating PIC layout, moving signals"
Comment4 ""
$EndDescr
$Comp
L distance_sensors:VL53L0X U3
U 1 1 5DBAFB11
P 7000 1600
F 0 "U3" H 7350 800 50  0000 C CNN
F 1 "VL53L0X" H 6750 2000 50  0000 C CNN
F 2 "footprints:VL53L0X" H 7000 1500 50  0001 C CNN
F 3 "" H 7000 1500 50  0001 C CNN
	1    7000 1600
	1    0    0    -1  
$EndComp
Text Label 9400 3950 2    50   ~ 0
SDA1
Text Label 9400 4050 2    50   ~ 0
SCL1
Text Label 9400 4650 2    50   ~ 0
SDA2_LV
$Comp
L Device:R R4
U 1 1 5DBB5C91
P 9500 3800
F 0 "R4" H 9550 3850 50  0000 L CNN
F 1 "2.2kR" H 9550 3750 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9430 3800 50  0001 C CNN
F 3 "~" H 9500 3800 50  0001 C CNN
	1    9500 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5DBB5CC1
P 9850 3900
F 0 "R6" H 9900 3950 50  0000 L CNN
F 1 "2.2kR" H 9900 3850 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9780 3900 50  0001 C CNN
F 3 "~" H 9850 3900 50  0001 C CNN
	1    9850 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5DBB5EA4
P 9500 4400
F 0 "R5" H 9550 4350 50  0000 L CNN
F 1 "2.2kR" H 9550 4450 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9430 4400 50  0001 C CNN
F 3 "~" H 9500 4400 50  0001 C CNN
	1    9500 4400
	1    0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 5DBB5EAB
P 9850 4500
F 0 "R8" H 9900 4450 50  0000 L CNN
F 1 "2.2kR" H 9900 4550 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9780 4500 50  0001 C CNN
F 3 "~" H 9850 4500 50  0001 C CNN
	1    9850 4500
	1    0    0    1   
$EndComp
Text Notes 5850 750  0    50   ~ 0
LEFT
Text Label 6050 1650 0    50   ~ 0
SDA1
Text Label 6050 1800 0    50   ~ 0
SCL1
Text Label 8850 1650 0    50   ~ 0
SDA1
Text Label 8850 1800 0    50   ~ 0
SCL1
$Comp
L power:VDD #PWR015
U 1 1 5DBB646D
P 9500 3650
F 0 "#PWR015" H 9500 3500 50  0001 C CNN
F 1 "VDD" H 9517 3823 50  0000 C CNN
F 2 "" H 9500 3650 50  0001 C CNN
F 3 "" H 9500 3650 50  0001 C CNN
	1    9500 3650
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR017
U 1 1 5DBB648D
P 9850 3750
F 0 "#PWR017" H 9850 3600 50  0001 C CNN
F 1 "VDD" H 9867 3923 50  0000 C CNN
F 2 "" H 9850 3750 50  0001 C CNN
F 3 "" H 9850 3750 50  0001 C CNN
	1    9850 3750
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR016
U 1 1 5DBB64A6
P 9500 4250
F 0 "#PWR016" H 9500 4100 50  0001 C CNN
F 1 "VDD" H 9500 4400 50  0000 C CNN
F 2 "" H 9500 4250 50  0001 C CNN
F 3 "" H 9500 4250 50  0001 C CNN
	1    9500 4250
	-1   0    0    -1  
$EndComp
$Comp
L power:VDD #PWR018
U 1 1 5DBB64CD
P 9850 4350
F 0 "#PWR018" H 9850 4200 50  0001 C CNN
F 1 "VDD" H 9850 4500 50  0000 C CNN
F 2 "" H 9850 4350 50  0001 C CNN
F 3 "" H 9850 4350 50  0001 C CNN
	1    9850 4350
	-1   0    0    -1  
$EndComp
Text Label 6050 1350 0    50   ~ 0
XSHUT_L
Text Label 8850 1350 0    50   ~ 0
XSHUT_R
Text Label 6050 1500 0    50   ~ 0
INT_L
Text Label 8850 1500 0    50   ~ 0
INT_R
Wire Wire Line
	8850 1800 9200 1800
Wire Wire Line
	8850 1650 9200 1650
Wire Wire Line
	6050 1800 6400 1800
Wire Wire Line
	6050 1650 6400 1650
$Comp
L power:GND #PWR010
U 1 1 5DBB70E7
P 4150 4550
F 0 "#PWR010" H 4150 4300 50  0001 C CNN
F 1 "GND" H 4155 4377 50  0000 C CNN
F 2 "" H 4150 4550 50  0001 C CNN
F 3 "" H 4150 4550 50  0001 C CNN
	1    4150 4550
	-1   0    0    -1  
$EndComp
Text Label 9400 3650 2    50   ~ 0
XSHUT_R
Text Label 9400 3550 2    50   ~ 0
INT_R
Text Label 9400 3350 2    50   ~ 0
XSHUT_L
Text Label 9400 3450 2    50   ~ 0
INT_L
$Comp
L Regulator_Linear:MIC5504-2.8YM5 U1
U 1 1 5DBDA457
P 3600 1150
F 0 "U1" H 3850 900 50  0000 C CNN
F 1 "MIC5504-2.8YM5" H 3600 1426 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5_HandSoldering" H 3600 750 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/MIC550X.pdf" H 3350 1400 50  0001 C CNN
	1    3600 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5DBE05CD
P 1750 1450
F 0 "#PWR01" H 1750 1200 50  0001 C CNN
F 1 "GND" H 1755 1277 50  0000 C CNN
F 2 "" H 1750 1450 50  0001 C CNN
F 3 "" H 1750 1450 50  0001 C CNN
	1    1750 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5DBE061C
P 3600 1650
F 0 "#PWR07" H 3600 1400 50  0001 C CNN
F 1 "GND" H 3605 1477 50  0000 C CNN
F 2 "" H 3600 1650 50  0001 C CNN
F 3 "" H 3600 1650 50  0001 C CNN
	1    3600 1650
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR09
U 1 1 5DBE150C
P 4200 3200
F 0 "#PWR09" H 4200 3050 50  0001 C CNN
F 1 "VDD" H 4217 3373 50  0000 C CNN
F 2 "" H 4200 3200 50  0001 C CNN
F 3 "" H 4200 3200 50  0001 C CNN
	1    4200 3200
	-1   0    0    -1  
$EndComp
$Comp
L power:VDD #PWR08
U 1 1 5DBE16CF
P 4400 1050
F 0 "#PWR08" H 4400 900 50  0001 C CNN
F 1 "VDD" H 4417 1223 50  0000 C CNN
F 2 "" H 4400 1050 50  0001 C CNN
F 3 "" H 4400 1050 50  0001 C CNN
	1    4400 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1150 2200 1150
Wire Wire Line
	1750 1250 2200 1250
Text Label 2200 1150 2    50   ~ 0
SCL2_HV
Text Label 2200 1250 2    50   ~ 0
SDA2_HV
Wire Wire Line
	3200 1050 3000 1050
Wire Wire Line
	3000 1050 3000 1250
Wire Wire Line
	3000 1250 3200 1250
Wire Wire Line
	3000 1250 3000 1350
Connection ~ 3000 1250
$Comp
L Device:C C1
U 1 1 5DBE3FCE
P 3000 1500
F 0 "C1" H 3115 1546 50  0000 L CNN
F 1 "1µF" H 3115 1455 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3038 1350 50  0001 C CNN
F 3 "~" H 3000 1500 50  0001 C CNN
	1    3000 1500
	1    0    0    -1  
$EndComp
Text Notes 2850 1850 0    50   ~ 0
X5R/X7R\nCeramic prefered
$Comp
L Device:C C2
U 1 1 5DBE4179
P 4150 1500
F 0 "C2" H 4350 1450 50  0000 R CNN
F 1 "1µF" H 4400 1550 50  0000 R CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4188 1350 50  0001 C CNN
F 3 "~" H 4150 1500 50  0001 C CNN
	1    4150 1500
	-1   0    0    1   
$EndComp
Text Notes 4000 1850 0    50   ~ 0
X5R/X7R\nCeramic prefered
Wire Wire Line
	4000 1050 4150 1050
Wire Wire Line
	4150 1050 4150 1350
Connection ~ 4150 1050
Wire Wire Line
	4150 1050 4400 1050
Wire Wire Line
	3000 1650 3600 1650
Connection ~ 3600 1650
Wire Wire Line
	3600 1650 4150 1650
Wire Wire Line
	3600 1450 3600 1650
Connection ~ 3000 1050
$Comp
L Interface:PCA9306 U5
U 1 1 5DBE9A8C
P 1350 4500
F 0 "U5" H 1600 4050 50  0000 C CNN
F 1 "PCA9306" H 1000 4950 50  0000 C CNN
F 2 "Housings_SSOP:SSOP-8_2.95x2.8mm_Pitch0.65mm" H 950 4850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/pca9306.pdf" H 1050 4950 50  0001 C CNN
	1    1350 4500
	1    0    0    -1  
$EndComp
Text Notes 1800 5150 0    50   ~ 0
VREF1 Low Voltage\nVREF2 High Voltage\n
Text Notes 5550 600  0    50   ~ 0
TOF Sensors
Text Notes 950  900  0    50   ~ 0
Interface Header
Text Notes 2850 700  0    50   ~ 0
LDO Voltage Regulator
Text Notes 600  3400 0    50   ~ 0
I2C Voltage translator
Wire Wire Line
	1750 4500 2150 4500
Wire Wire Line
	1750 4600 2150 4600
Text Label 2150 4500 2    50   ~ 0
SCL2_HV
Text Label 2150 4600 2    50   ~ 0
SDA2_HV
Wire Wire Line
	950  4500 600  4500
Wire Wire Line
	600  4600 950  4600
Text Label 600  4500 0    50   ~ 0
SCL2_LV
Text Label 600  4600 0    50   ~ 0
SDA2_LV
$Comp
L Device:R R11
U 1 1 5DBF29CC
P 1900 3800
F 0 "R11" V 1693 3800 50  0000 C CNN
F 1 "200kR" V 1784 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 1830 3800 50  0001 C CNN
F 3 "~" H 1900 3800 50  0001 C CNN
	1    1900 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 4000 1750 4000
Wire Wire Line
	1750 4400 1750 4250
Connection ~ 1750 4000
$Comp
L power:VDD #PWR021
U 1 1 5DBF59BD
P 1250 3800
F 0 "#PWR021" H 1250 3650 50  0001 C CNN
F 1 "VDD" H 1268 3973 50  0000 C CNN
F 2 "" H 1250 3800 50  0001 C CNN
F 3 "" H 1250 3800 50  0001 C CNN
	1    1250 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5DBF5ABB
P 1350 5000
F 0 "#PWR022" H 1350 4750 50  0001 C CNN
F 1 "GND" H 1355 4827 50  0000 C CNN
F 2 "" H 1350 5000 50  0001 C CNN
F 3 "" H 1350 5000 50  0001 C CNN
	1    1350 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 4000 1250 3850
Wire Wire Line
	1250 3850 1150 3850
Connection ~ 1250 3850
Wire Wire Line
	1250 3850 1250 3800
$Comp
L Device:R R10
U 1 1 5DBF73C0
P 1000 3850
F 0 "R10" V 793 3850 50  0000 C CNN
F 1 "262.5kR" V 884 3850 50  0000 C CIN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 930 3850 50  0001 C CNN
F 3 "~" H 1000 3850 50  0001 C CNN
	1    1000 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	850  3850 700  3850
Wire Wire Line
	700  3850 700  4000
$Comp
L power:GND #PWR020
U 1 1 5DBF8162
P 700 4000
F 0 "#PWR020" H 700 3750 50  0001 C CNN
F 1 "GND" H 705 3827 50  0000 C CNN
F 2 "" H 700 4000 50  0001 C CNN
F 3 "" H 700 4000 50  0001 C CNN
	1    700  4000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5DBF8379
P 2250 4000
F 0 "C7" V 2350 4100 50  0000 C CNN
F 1 "0.1µF" V 2150 3850 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2250 4000 50  0001 C CNN
F 3 "~" H 2250 4000 50  0001 C CNN
	1    2250 4000
	0    1    1    0   
$EndComp
$Comp
L Device:C C8
U 1 1 5DBF8433
P 2250 4250
F 0 "C8" V 2350 4350 50  0000 C CNN
F 1 "4.7µF" V 2150 4100 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2288 4100 50  0001 C CNN
F 3 "~" H 2250 4250 50  0001 C CNN
	1    2250 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	1750 4000 1750 3800
Wire Wire Line
	1750 4000 2150 4000
Wire Wire Line
	2100 4250 1750 4250
Connection ~ 1750 4250
Wire Wire Line
	1750 4250 1750 4000
Wire Wire Line
	2350 4000 2500 4000
Wire Wire Line
	2500 4000 2500 4250
Wire Wire Line
	2500 4250 2400 4250
Wire Wire Line
	2500 4250 2500 4350
Connection ~ 2500 4250
$Comp
L power:GND #PWR026
U 1 1 5DBFD178
P 2500 4350
F 0 "#PWR026" H 2500 4100 50  0001 C CNN
F 1 "GND" H 2505 4177 50  0000 C CNN
F 2 "" H 2500 4350 50  0001 C CNN
F 3 "" H 2500 4350 50  0001 C CNN
	1    2500 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 1350 7650 1350
Connection ~ 7650 1350
Wire Wire Line
	7650 1950 7600 1950
$Comp
L Device:C C5
U 1 1 5DC09E47
P 8150 1550
F 0 "C5" H 8200 1450 50  0000 L CNN
F 1 "4.7µF" H 8200 1650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 8188 1400 50  0001 C CNN
F 3 "~" H 8150 1550 50  0001 C CNN
	1    8150 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5DC09EAC
P 7900 1550
F 0 "C4" H 7850 1450 50  0000 R CNN
F 1 "0.1µF" H 7900 1650 50  0000 R CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 7900 1550 50  0001 C CNN
F 3 "~" H 7900 1550 50  0001 C CNN
	1    7900 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 1650 7650 1800
Wire Wire Line
	7600 1800 7650 1800
Connection ~ 7650 1800
Wire Wire Line
	7650 1800 7650 1950
Wire Wire Line
	7650 1950 7650 2100
Wire Wire Line
	7650 2100 7600 2100
Connection ~ 7650 1950
Wire Wire Line
	7650 2100 7650 2250
Wire Wire Line
	7650 2250 7600 2250
Connection ~ 7650 2100
Wire Wire Line
	7600 1650 7650 1650
Wire Wire Line
	7650 1800 7900 1800
Wire Wire Line
	7650 1500 7600 1500
Wire Wire Line
	7650 1350 7650 1500
Wire Wire Line
	8150 1700 8150 1800
Wire Wire Line
	8150 1400 8150 1350
Wire Wire Line
	7650 1350 7900 1350
Wire Wire Line
	7900 1450 7900 1350
Connection ~ 7900 1350
Wire Wire Line
	7900 1350 8150 1350
Wire Wire Line
	7900 1650 7900 1800
Connection ~ 7900 1800
Wire Wire Line
	7900 1800 8150 1800
$Comp
L power:GND #PWR014
U 1 1 5DC27691
P 8150 1800
F 0 "#PWR014" H 8150 1550 50  0001 C CNN
F 1 "GND" H 8155 1627 50  0000 C CNN
F 2 "" H 8150 1800 50  0001 C CNN
F 3 "" H 8150 1800 50  0001 C CNN
	1    8150 1800
	1    0    0    -1  
$EndComp
Connection ~ 8150 1800
$Comp
L distance_sensors:VL53L0X U4
U 1 1 5DC28D88
P 9800 1600
F 0 "U4" H 10150 800 50  0000 C CNN
F 1 "VL53L0X" H 9550 2000 50  0000 C CNN
F 2 "footprints:VL53L0X" H 9800 1500 50  0001 C CNN
F 3 "" H 9800 1500 50  0001 C CNN
	1    9800 1600
	1    0    0    -1  
$EndComp
Text Notes 8600 750  0    50   ~ 0
RIGHT
Wire Wire Line
	10400 1350 10450 1350
Connection ~ 10450 1350
Wire Wire Line
	10450 1950 10400 1950
$Comp
L Device:C C9
U 1 1 5DC28D93
P 10950 1550
F 0 "C9" H 11000 1450 50  0000 L CNN
F 1 "4.7µF" H 10950 1650 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10988 1400 50  0001 C CNN
F 3 "~" H 10950 1550 50  0001 C CNN
	1    10950 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5DC28D9A
P 10700 1550
F 0 "C6" H 10650 1450 50  0000 R CNN
F 1 "0.1µF" H 10700 1650 50  0000 R CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10700 1550 50  0001 C CNN
F 3 "~" H 10700 1550 50  0001 C CNN
	1    10700 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 1650 10450 1800
Wire Wire Line
	10400 1800 10450 1800
Connection ~ 10450 1800
Wire Wire Line
	10450 1800 10450 1950
Wire Wire Line
	10450 1950 10450 2100
Wire Wire Line
	10450 2100 10400 2100
Connection ~ 10450 1950
Wire Wire Line
	10450 2100 10450 2250
Wire Wire Line
	10450 2250 10400 2250
Connection ~ 10450 2100
Wire Wire Line
	10400 1650 10450 1650
Wire Wire Line
	10450 1800 10700 1800
Wire Wire Line
	10450 1500 10400 1500
Wire Wire Line
	10450 1350 10450 1500
Wire Wire Line
	10950 1700 10950 1800
Wire Wire Line
	10950 1400 10950 1350
Wire Wire Line
	10450 1350 10700 1350
Wire Wire Line
	10700 1450 10700 1350
Connection ~ 10700 1350
Wire Wire Line
	10700 1350 10950 1350
Wire Wire Line
	10700 1650 10700 1800
Connection ~ 10700 1800
Wire Wire Line
	10700 1800 10950 1800
$Comp
L power:GND #PWR025
U 1 1 5DC28DB8
P 10950 1800
F 0 "#PWR025" H 10950 1550 50  0001 C CNN
F 1 "GND" H 10955 1627 50  0000 C CNN
F 2 "" H 10950 1800 50  0001 C CNN
F 3 "" H 10950 1800 50  0001 C CNN
	1    10950 1800
	1    0    0    -1  
$EndComp
Connection ~ 10950 1800
$Comp
L power:VDD #PWR013
U 1 1 5DC32765
P 7900 1350
F 0 "#PWR013" H 7900 1200 50  0001 C CNN
F 1 "VDD" H 7917 1523 50  0000 C CNN
F 2 "" H 7900 1350 50  0001 C CNN
F 3 "" H 7900 1350 50  0001 C CNN
	1    7900 1350
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR024
U 1 1 5DC32810
P 10700 1350
F 0 "#PWR024" H 10700 1200 50  0001 C CNN
F 1 "VDD" H 10717 1523 50  0000 C CNN
F 2 "" H 10700 1350 50  0001 C CNN
F 3 "" H 10700 1350 50  0001 C CNN
	1    10700 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5DC389E7
P 6000 1150
F 0 "R3" H 6050 1100 50  0000 L CNN
F 1 "10kR" H 6050 1200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5930 1150 50  0001 C CNN
F 3 "~" H 6000 1150 50  0001 C CNN
	1    6000 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5DC38A5D
P 5800 1300
F 0 "R2" H 5750 1250 50  0000 R CNN
F 1 "10kR" H 5750 1350 50  0000 R CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5730 1300 50  0001 C CNN
F 3 "~" H 5800 1300 50  0001 C CNN
	1    5800 1300
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR011
U 1 1 5DC3D23C
P 5900 1000
F 0 "#PWR011" H 5900 850 50  0001 C CNN
F 1 "VDD" H 5917 1173 50  0000 C CNN
F 2 "" H 5900 1000 50  0001 C CNN
F 3 "" H 5900 1000 50  0001 C CNN
	1    5900 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5DC3D399
P 8800 1150
F 0 "R9" H 8950 1100 50  0000 R CNN
F 1 "10kR" H 9000 1300 50  0000 R CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 8730 1150 50  0001 C CNN
F 3 "~" H 8800 1150 50  0001 C CNN
	1    8800 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5DC3D3A0
P 8600 1300
F 0 "R7" H 8450 1250 50  0000 L CNN
F 1 "10kR" H 8400 1450 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 8530 1300 50  0001 C CNN
F 3 "~" H 8600 1300 50  0001 C CNN
	1    8600 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 1150 8600 1000
Wire Wire Line
	8600 1000 8700 1000
Connection ~ 8700 1000
Wire Wire Line
	8700 1000 8800 1000
$Comp
L power:VDD #PWR019
U 1 1 5DC3D3AD
P 8700 1000
F 0 "#PWR019" H 8700 850 50  0001 C CNN
F 1 "VDD" H 8717 1173 50  0000 C CNN
F 2 "" H 8700 1000 50  0001 C CNN
F 3 "" H 8700 1000 50  0001 C CNN
	1    8700 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5DC91043
P 4050 3250
F 0 "C3" V 4000 3150 50  0000 C CNN
F 1 "0.1µF" V 3900 3250 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4088 3100 50  0001 C CNN
F 3 "~" H 4050 3250 50  0001 C CNN
	1    4050 3250
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5DC9EB4D
P 3900 3250
F 0 "#PWR012" H 3900 3000 50  0001 C CNN
F 1 "GND" H 3900 3300 50  0000 C CNN
F 2 "" H 3900 3250 50  0001 C CNN
F 3 "" H 3900 3250 50  0001 C CNN
	1    3900 3250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5800 1500 5800 1450
Wire Wire Line
	6000 1350 6000 1300
Wire Wire Line
	5800 1150 5800 1000
Wire Wire Line
	5800 1000 5900 1000
Connection ~ 5900 1000
Wire Wire Line
	5900 1000 6000 1000
Wire Wire Line
	5800 1500 6400 1500
Wire Wire Line
	6000 1350 6400 1350
Wire Wire Line
	8800 1350 8800 1300
Wire Wire Line
	8800 1350 9200 1350
Wire Wire Line
	8600 1450 8600 1500
Wire Wire Line
	8600 1500 9200 1500
Wire Notes Line
	5450 2500 11200 2500
Wire Wire Line
	9000 3450 9400 3450
$Comp
L Connector:Conn_01x06_Male J1
U 1 1 5DD8B607
P 1200 2400
F 0 "J1" H 1300 2000 50  0000 C CNN
F 1 "Conn_01x06_Male" H 1200 2750 50  0000 C CNN
F 2 "footprints:Conn_01x06_2.54mm_Horizontal_SMD_Handsoldered" H 1200 2400 50  0001 C CNN
F 3 "~" H 1200 2400 50  0001 C CNN
	1    1200 2400
	1    0    0    -1  
$EndComp
Text Notes 600  2250 0    50   ~ 0
Vpp / MCLR
Text Notes 600  2350 0    50   ~ 0
Vdd
Text Notes 600  2450 0    50   ~ 0
Ground
Text Notes 600  2550 0    50   ~ 0
ICSPDAT
Text Notes 600  2650 0    50   ~ 0
ICSPCLK
Text Notes 600  2750 0    50   ~ 0
NC
Wire Wire Line
	1400 2300 1800 2300
Wire Wire Line
	1800 2300 1800 2050
Wire Wire Line
	1400 2400 1800 2400
Wire Wire Line
	1800 2400 1800 2900
$Comp
L power:GND #PWR03
U 1 1 5DD9EE68
P 1800 2900
F 0 "#PWR03" H 1800 2650 50  0001 C CNN
F 1 "GND" H 1805 2727 50  0000 C CNN
F 2 "" H 1800 2900 50  0001 C CNN
F 3 "" H 1800 2900 50  0001 C CNN
	1    1800 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2500 1750 2500
Wire Wire Line
	1400 2600 1750 2600
NoConn ~ 1400 2700
Text Label 1750 2500 2    50   ~ 0
ICSPDAT
Text Label 1750 2600 2    50   ~ 0
ICSPCLK
Text Label 9400 4850 2    50   ~ 0
ICSPDAT
Text Label 9400 4750 2    50   ~ 0
ICSPCLK
Wire Notes Line
	1900 1750 1900 3200
Wire Notes Line
	1900 3200 550  3200
Wire Notes Line
	550  3200 550  1750
Wire Notes Line
	550  1750 1900 1750
Text Notes 600  1850 0    50   ~ 0
ICSP Header
Wire Notes Line
	2800 600  2800 1900
Wire Notes Line
	2800 1900 4700 1900
Wire Notes Line
	4700 1900 4700 600 
Wire Notes Line
	4700 600  2800 600 
Wire Notes Line
	550  3300 550  5300
Wire Notes Line
	550  5300 2600 5300
Wire Notes Line
	2600 5300 2600 3300
Wire Notes Line
	2600 3300 550  3300
$Comp
L power:VCC #PWR0102
U 1 1 5DC2BD05
P 2050 3800
F 0 "#PWR0102" H 2050 3650 50  0001 C CNN
F 1 "VCC" H 2067 3973 50  0000 C CNN
F 2 "" H 2050 3800 50  0001 C CNN
F 3 "" H 2050 3800 50  0001 C CNN
	1    2050 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 4850 9000 4850
Wire Wire Line
	9400 4750 9000 4750
$Comp
L power:VCC #PWR0101
U 1 1 5DC2BCB7
P 2450 1050
F 0 "#PWR0101" H 2450 900 50  0001 C CNN
F 1 "VCC" H 2467 1223 50  0000 C CNN
F 2 "" H 2450 1050 50  0001 C CNN
F 3 "" H 2450 1050 50  0001 C CNN
	1    2450 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1050 3000 1050
Wire Notes Line
	5450 2500 5450 500 
$Comp
L Device:C C11
U 1 1 5DDD141D
P 4050 3550
F 0 "C11" V 4100 3400 50  0000 C CNN
F 1 "0.1µF" V 3900 3550 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4088 3400 50  0001 C CNN
F 3 "~" H 4050 3550 50  0001 C CNN
	1    4050 3550
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5DDD14D6
P 3900 3550
F 0 "#PWR05" H 3900 3300 50  0001 C CNN
F 1 "GND" H 3900 3600 50  0000 C CNN
F 2 "" H 3900 3550 50  0001 C CNN
F 3 "" H 3900 3550 50  0001 C CNN
	1    3900 3550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4200 3200 4200 3250
Wire Wire Line
	9400 3650 9000 3650
Text Label 1750 2200 2    50   ~ 0
Vpp
Wire Wire Line
	1750 2200 1400 2200
Wire Wire Line
	9500 4550 9000 4550
Text Label 9400 4550 2    50   ~ 0
SCL2_LV
Wire Wire Line
	9000 3550 9400 3550
Connection ~ 4200 3250
NoConn ~ 4300 4000
NoConn ~ 4300 4100
NoConn ~ 19650 2850
Wire Wire Line
	9000 4650 9850 4650
Wire Wire Line
	9000 3350 9400 3350
Text Notes 4400 2950 2    50   ~ 0
Datasheet advices 0.1µF\n50V Ceramic capacitors
$Comp
L Device:C C12
U 1 1 5DF790B3
P 4300 5000
F 0 "C12" H 4415 5046 50  0000 L CNN
F 1 "10µF" H 4415 4955 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4338 4850 50  0001 C CNN
F 3 "~" H 4300 5000 50  0001 C CNN
	1    4300 5000
	1    0    0    -1  
$EndComp
Text Notes 4450 5300 0    50   ~ 0
Datasheet advice 10µF\n16V (or more) Ceramic capacitor\n
$Comp
L power:GND #PWR06
U 1 1 5DF7928E
P 4300 5150
F 0 "#PWR06" H 4300 4900 50  0001 C CNN
F 1 "GND" H 4305 4977 50  0000 C CNN
F 2 "" H 4300 5150 50  0001 C CNN
F 3 "" H 4300 5150 50  0001 C CNN
	1    4300 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1050 2450 1050
Connection ~ 2450 1050
NoConn ~ 4300 4200
Wire Notes Line
	5450 500  11200 500 
Wire Notes Line
	11200 500  11200 2500
$Comp
L Switch:SW_DIP_x03 SW1
U 1 1 5DE7D246
P 1750 6400
F 0 "SW1" H 1750 6867 50  0000 C CNN
F 1 "SW_DIP_x03" H 1750 6776 50  0000 C CNN
F 2 "footprints:DIP_Switch_CTS_219_Series_Gull_Wing_HandSoldered" H 1750 6400 50  0001 C CNN
F 3 "~" H 1750 6400 50  0001 C CNN
	1    1750 6400
	1    0    0    -1  
$EndComp
Text Notes 1250 5850 0    50   ~ 0
DIP1 used for calibration mode\nDIP2 and DIP3 reserved for now.
Wire Wire Line
	1450 6200 1350 6200
Wire Wire Line
	1350 6200 1350 6300
Wire Wire Line
	1350 6300 1450 6300
Wire Wire Line
	1350 6300 1350 6400
Wire Wire Line
	1350 6400 1450 6400
Connection ~ 1350 6300
$Comp
L power:VDD #PWR023
U 1 1 5DE89598
P 1350 6100
F 0 "#PWR023" H 1350 5950 50  0001 C CNN
F 1 "VDD" H 1367 6273 50  0000 C CNN
F 2 "" H 1350 6100 50  0001 C CNN
F 3 "" H 1350 6100 50  0001 C CNN
	1    1350 6100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1350 6100 1350 6200
Connection ~ 1350 6200
Wire Wire Line
	2050 6200 2700 6200
Text Label 2300 6200 2    50   ~ 0
DIP1
Text Label 2300 6300 2    50   ~ 0
DIP2
Text Label 2300 6400 2    50   ~ 0
DIP3
Wire Wire Line
	2300 6400 2050 6400
Wire Wire Line
	2050 6300 2500 6300
$Comp
L Device:R R13
U 1 1 5DEBB9AC
P 2500 6600
F 0 "R13" H 2550 6450 50  0000 C CNN
F 1 "10kR" V 2500 6600 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2430 6600 50  0001 C CNN
F 3 "~" H 2500 6600 50  0001 C CNN
	1    2500 6600
	-1   0    0    1   
$EndComp
$Comp
L Device:R R14
U 1 1 5DEBBDAC
P 2700 6600
F 0 "R14" H 2750 6450 50  0000 C CNN
F 1 "10kR" V 2700 6600 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2630 6600 50  0001 C CNN
F 3 "~" H 2700 6600 50  0001 C CNN
	1    2700 6600
	-1   0    0    1   
$EndComp
$Comp
L Device:R R12
U 1 1 5DEBA899
P 2300 6600
F 0 "R12" H 2350 6450 50  0000 C CNN
F 1 "10kR" V 2300 6600 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2230 6600 50  0001 C CNN
F 3 "~" H 2300 6600 50  0001 C CNN
	1    2300 6600
	-1   0    0    1   
$EndComp
Wire Wire Line
	2700 6450 2700 6200
Wire Wire Line
	2500 6300 2500 6450
Wire Wire Line
	2300 6450 2300 6400
$Comp
L power:GND #PWR027
U 1 1 5DED773D
P 2500 6750
F 0 "#PWR027" H 2500 6500 50  0001 C CNN
F 1 "GND" H 2505 6577 50  0000 C CNN
F 2 "" H 2500 6750 50  0001 C CNN
F 3 "" H 2500 6750 50  0001 C CNN
	1    2500 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 6750 2500 6750
Connection ~ 2500 6750
Wire Wire Line
	2500 6750 2300 6750
Wire Wire Line
	4300 4450 4300 4550
Text Label 4150 3850 2    50   ~ 0
Vpp
$Comp
L Device:D_Schottky D1
U 1 1 5DC31335
P 3750 3700
F 0 "D1" H 3800 3600 50  0000 R CNN
F 1 "D_Schottky" H 3950 3800 50  0000 R CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 3750 3700 50  0001 C CNN
F 3 "~" H 3750 3700 50  0001 C CNN
	1    3750 3700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5DC310B2
P 3450 3900
F 0 "#PWR0103" H 3450 3650 50  0001 C CNN
F 1 "GND" H 3455 3727 50  0000 C CNN
F 2 "" H 3450 3900 50  0001 C CNN
F 3 "" H 3450 3900 50  0001 C CNN
	1    3450 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5DC23234
P 3450 3800
F 0 "C10" H 3300 3700 50  0000 L CNN
F 1 "0.1µF" H 3150 3800 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3450 3800 50  0001 C CNN
F 3 "~" H 3450 3800 50  0001 C CNN
	1    3450 3800
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR04
U 1 1 5DD5985D
P 3450 3400
F 0 "#PWR04" H 3450 3250 50  0001 C CNN
F 1 "VDD" H 3467 3573 50  0000 C CNN
F 2 "" H 3450 3400 50  0001 C CNN
F 3 "" H 3450 3400 50  0001 C CNN
	1    3450 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5DC78F66
P 3450 3550
F 0 "R1" V 3550 3550 50  0000 C CNN
F 1 "10kR" V 3350 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3380 3550 50  0001 C CNN
F 3 "~" H 3450 3550 50  0001 C CNN
	1    3450 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 3700 3450 3700
Connection ~ 3450 3700
Wire Wire Line
	4200 3250 4200 3350
Wire Wire Line
	3900 3700 4000 3700
Wire Wire Line
	4300 3450 4200 3450
Connection ~ 4200 3450
Wire Wire Line
	4200 3450 4200 3550
Wire Wire Line
	4300 3350 4200 3350
Connection ~ 4200 3350
Wire Wire Line
	4200 3350 4200 3450
NoConn ~ 4300 3800
NoConn ~ 4300 3900
Connection ~ 4300 4550
Wire Wire Line
	4300 4550 4300 4650
Wire Wire Line
	4300 4550 4150 4550
Wire Wire Line
	9000 3950 9500 3950
Wire Wire Line
	9000 4050 9850 4050
Wire Wire Line
	9000 4350 9400 4350
Wire Wire Line
	9400 4250 9000 4250
Wire Wire Line
	9000 4150 9400 4150
Text Label 9400 4350 2    50   ~ 0
DIP1
Text Label 9400 4250 2    50   ~ 0
DIP2
Text Label 9400 4150 2    50   ~ 0
DIP3
NoConn ~ 9000 3850
Wire Notes Line
	2950 5450 2950 2650
Wire Notes Line
	2950 2650 10150 2650
Wire Notes Line
	10150 2650 10150 5450
Wire Notes Line
	2950 5450 10150 5450
Text Notes 2950 2750 0    50   ~ 0
Microcontroller
Wire Notes Line
	2850 5500 2850 7000
Wire Notes Line
	2850 7000 550  7000
Wire Notes Line
	550  7000 550  5500
Wire Notes Line
	550  5500 2850 5500
Text Notes 550  5600 0    50   ~ 0
Configuration switches
Wire Wire Line
	9000 3750 9400 3750
Text Label 9400 3750 2    50   ~ 0
INT_OUT_VDD
$Comp
L Connector_Generic:Conn_01x05 J2
U 1 1 5E1C2121
P 1550 1250
F 0 "J2" H 1468 1667 50  0000 C CNN
F 1 "Conn_01x05" H 1468 1576 50  0000 C CNN
F 2 "footprints:SolderWirePad_01x05_2-5mmDrill_HandSoldering" H 1550 1250 50  0001 C CNN
F 3 "~" H 1550 1250 50  0001 C CNN
	1    1550 1250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1750 1350 2200 1350
Text Label 2200 1350 2    50   ~ 0
INT_OUT_VCC
Wire Wire Line
	9000 4450 9400 4450
Text Label 9400 4450 2    50   ~ 0
LED
Text Label 8550 5200 0    50   ~ 0
LED
Wire Wire Line
	8550 5200 8750 5200
$Comp
L Device:LED D2
U 1 1 5DEB4926
P 8900 5200
F 0 "D2" H 8892 4945 50  0000 C CNN
F 1 "LED" H 8892 5036 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 8900 5200 50  0001 C CNN
F 3 "~" H 8900 5200 50  0001 C CNN
	1    8900 5200
	-1   0    0    1   
$EndComp
Wire Wire Line
	9050 5200 9150 5200
$Comp
L Device:R R15
U 1 1 5DEBA798
P 9300 5200
F 0 "R15" V 9400 5150 50  0000 L CNN
F 1 "47R" V 9300 5100 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9230 5200 50  0001 C CNN
F 3 "~" H 9300 5200 50  0001 C CNN
	1    9300 5200
	0    -1   1    0   
$EndComp
Wire Wire Line
	9450 5200 9550 5200
$Comp
L power:GND #PWR028
U 1 1 5DEC0A22
P 9550 5200
F 0 "#PWR028" H 9550 4950 50  0001 C CNN
F 1 "GND" H 9555 5027 50  0000 C CNN
F 2 "" H 9550 5200 50  0001 C CNN
F 3 "" H 9550 5200 50  0001 C CNN
	1    9550 5200
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP1
U 1 1 5DEB60C3
P 7650 5850
F 0 "TP1" H 7803 5951 50  0000 L CNN
F 1 "TestPoint_Probe" H 7803 5860 50  0000 L CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 7850 5850 50  0001 C CNN
F 3 "~" H 7850 5850 50  0001 C CNN
	1    7650 5850
	1    0    0    -1  
$EndComp
Text Label 7650 6050 2    50   ~ 0
SDA1
Text Label 8800 5850 2    50   ~ 0
SCL1
Text Label 8800 6050 2    50   ~ 0
SDA2_LV
Text Label 8800 6250 2    50   ~ 0
SCL2_LV
$Comp
L power:GND #PWR030
U 1 1 5DEC048A
P 7650 6250
F 0 "#PWR030" H 7650 6000 50  0001 C CNN
F 1 "GND" H 7655 6077 50  0000 C CNN
F 2 "" H 7650 6250 50  0001 C CNN
F 3 "" H 7650 6250 50  0001 C CNN
	1    7650 6250
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP2
U 1 1 5DEC283E
P 7650 6050
F 0 "TP2" H 7803 6151 50  0000 L CNN
F 1 "TestPoint_Probe" H 7803 6060 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 7850 6050 50  0001 C CNN
F 3 "~" H 7850 6050 50  0001 C CNN
	1    7650 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP3
U 1 1 5DEC3B67
P 8800 5850
F 0 "TP3" H 8953 5951 50  0000 L CNN
F 1 "TestPoint_Probe" H 8953 5860 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 9000 5850 50  0001 C CNN
F 3 "~" H 9000 5850 50  0001 C CNN
	1    8800 5850
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP4
U 1 1 5DEC3D7D
P 8800 6050
F 0 "TP4" H 8953 6151 50  0000 L CNN
F 1 "TestPoint_Probe" H 8953 6060 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 9000 6050 50  0001 C CNN
F 3 "~" H 9000 6050 50  0001 C CNN
	1    8800 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP5
U 1 1 5DEC3FAA
P 8800 6250
F 0 "TP5" H 8953 6351 50  0000 L CNN
F 1 "TestPoint_Probe" H 8953 6260 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 9000 6250 50  0001 C CNN
F 3 "~" H 9000 6250 50  0001 C CNN
	1    8800 6250
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP8
U 1 1 5DEF33BD
P 7650 6250
F 0 "TP8" H 7803 6351 50  0000 L CNN
F 1 "TestPoint_Probe" H 7803 6260 50  0000 L CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 7850 6250 50  0001 C CNN
F 3 "~" H 7850 6250 50  0001 C CNN
	1    7650 6250
	1    0    0    -1  
$EndComp
Text Label 10000 6450 2    50   ~ 0
XSHUT_R
Text Label 10000 6250 2    50   ~ 0
INT_R
Text Label 10000 5850 2    50   ~ 0
XSHUT_L
Text Label 10000 6050 2    50   ~ 0
INT_L
$Comp
L Connector:TestPoint_Probe TP9
U 1 1 5DF2AF30
P 10000 5850
F 0 "TP9" H 10153 5951 50  0000 L CNN
F 1 "TestPoint_Probe" H 10153 5860 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 10200 5850 50  0001 C CNN
F 3 "~" H 10200 5850 50  0001 C CNN
	1    10000 5850
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP10
U 1 1 5DF2B72E
P 10000 6050
F 0 "TP10" H 10153 6151 50  0000 L CNN
F 1 "TestPoint_Probe" H 10153 6060 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 10200 6050 50  0001 C CNN
F 3 "~" H 10200 6050 50  0001 C CNN
	1    10000 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP11
U 1 1 5DF2BA31
P 10000 6250
F 0 "TP11" H 10153 6351 50  0000 L CNN
F 1 "TestPoint_Probe" H 10153 6260 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 10200 6250 50  0001 C CNN
F 3 "~" H 10200 6250 50  0001 C CNN
	1    10000 6250
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP12
U 1 1 5DF2BD85
P 10000 6450
F 0 "TP12" H 10153 6551 50  0000 L CNN
F 1 "TestPoint_Probe" H 10153 6460 50  0000 L CNN
F 2 "Measurement_Points:Test_Point_Keystone_5010-5014_Multipurpose" H 10200 6450 50  0001 C CNN
F 3 "~" H 10200 6450 50  0001 C CNN
	1    10000 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3850 4000 3700
Wire Wire Line
	4000 3850 4150 3850
Connection ~ 4000 3700
Wire Wire Line
	4000 3700 4300 3700
$Comp
L power:VDD #PWR029
U 1 1 5DFF4680
P 7650 5850
F 0 "#PWR029" H 7650 5700 50  0001 C CNN
F 1 "VDD" H 7667 6023 50  0000 C CNN
F 2 "" H 7650 5850 50  0001 C CNN
F 3 "" H 7650 5850 50  0001 C CNN
	1    7650 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GSD Q1
U 1 1 5DF4A0D4
P 4200 6250
F 0 "Q1" H 4404 6204 50  0000 L CNN
F 1 "BSH202,215" H 4404 6295 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 4400 6350 50  0001 C CNN
F 3 "~" H 4200 6250 50  0001 C CNN
	1    4200 6250
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5DF7FFB2
P 4300 6850
F 0 "#PWR033" H 4300 6600 50  0001 C CNN
F 1 "GND" H 4305 6677 50  0000 C CNN
F 2 "" H 4300 6850 50  0001 C CNN
F 3 "" H 4300 6850 50  0001 C CNN
	1    4300 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 5DF69975
P 4300 6700
F 0 "R18" H 4400 6700 50  0000 C CNN
F 1 "10kR" V 4300 6700 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4230 6700 50  0001 C CNN
F 3 "~" H 4300 6700 50  0001 C CNN
	1    4300 6700
	-1   0    0    1   
$EndComp
Wire Wire Line
	3100 6250 3600 6250
Connection ~ 3600 6250
Wire Wire Line
	3700 6250 3600 6250
$Comp
L Device:R R17
U 1 1 5DF4CAB4
P 3850 6250
F 0 "R17" V 3950 6200 50  0000 L CNN
F 1 "330R" V 3850 6150 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3780 6250 50  0001 C CNN
F 3 "~" H 3850 6250 50  0001 C CNN
	1    3850 6250
	0    -1   1    0   
$EndComp
Text Label 5100 6500 2    50   ~ 0
INT_OUT_VCC
Text Label 3100 6250 0    50   ~ 0
INT_OUT_VDD
$Comp
L power:GND #PWR031
U 1 1 5DF80FD1
P 3600 6600
F 0 "#PWR031" H 3600 6350 50  0001 C CNN
F 1 "GND" H 3605 6427 50  0000 C CNN
F 2 "" H 3600 6600 50  0001 C CNN
F 3 "" H 3600 6600 50  0001 C CNN
	1    3600 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6250 3600 6300
$Comp
L Device:R R16
U 1 1 5DF4D826
P 3600 6450
F 0 "R16" H 3700 6450 50  0000 C CNN
F 1 "200kR" V 3484 6450 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3530 6450 50  0001 C CNN
F 3 "~" H 3600 6450 50  0001 C CNN
	1    3600 6450
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR032
U 1 1 5DFC1BD2
P 4300 6050
F 0 "#PWR032" H 4300 5900 50  0001 C CNN
F 1 "VCC" H 4317 6223 50  0000 C CNN
F 2 "" H 4300 6050 50  0001 C CNN
F 3 "" H 4300 6050 50  0001 C CNN
	1    4300 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 6450 4300 6500
$Comp
L Device:R R19
U 1 1 5DFC34FE
P 4500 6500
F 0 "R19" V 4600 6450 50  0000 L CNN
F 1 "330R" V 4500 6400 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4430 6500 50  0001 C CNN
F 3 "~" H 4500 6500 50  0001 C CNN
	1    4500 6500
	0    -1   1    0   
$EndComp
Wire Wire Line
	4350 6500 4300 6500
Connection ~ 4300 6500
Wire Wire Line
	4300 6500 4300 6550
Wire Wire Line
	4650 6500 5100 6500
Wire Notes Line
	3000 5750 5200 5750
Wire Notes Line
	5200 5750 5200 7100
Wire Notes Line
	5200 7100 3000 7100
Wire Notes Line
	3000 7100 3000 5750
Text Notes 3050 5850 0    50   ~ 0
Interrupt Drive
Wire Notes Line
	10800 6500 10800 5500
Wire Notes Line
	10800 5500 7400 5500
Wire Notes Line
	7400 5500 7400 6500
Wire Notes Line
	7400 6500 10800 6500
Text Notes 7450 5600 0    50   ~ 0
Test Points
$Comp
L PIC24FJ64GA702:PIC24FJ64GA002 U2
U 1 1 5DEF71EE
P 4600 3250
F 0 "U2" H 6650 3437 60  0000 C CNN
F 1 "PIC24FJ64GA002" H 6650 3331 60  0000 C CNN
F 2 "Package_SO:SOIC-28W_7.5x17.9mm_P1.27mm" H 4900 1850 60  0001 C CNN
F 3 "" H 5100 2500 60  0000 C CNN
	1    4600 3250
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5E633983
P 1800 2050
F 0 "#PWR?" H 1800 1900 50  0001 C CNN
F 1 "VCC" H 1817 2223 50  0000 C CNN
F 2 "" H 1800 2050 50  0001 C CNN
F 3 "" H 1800 2050 50  0001 C CNN
	1    1800 2050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
