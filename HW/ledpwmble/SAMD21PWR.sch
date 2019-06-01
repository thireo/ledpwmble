EESchema Schematic File Version 4
LIBS:ledpwmble-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 3
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
L power:VCC #PWR046
U 1 1 5B0514A0
P 3375 2850
F 0 "#PWR046" H 3375 2700 50  0001 C CNN
F 1 "VCC" H 3392 3023 50  0000 C CNN
F 2 "" H 3375 2850 50  0001 C CNN
F 3 "" H 3375 2850 50  0001 C CNN
	1    3375 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead_Small L1
U 1 1 5B0514A6
P 3625 2900
F 0 "L1" V 3388 2900 50  0000 C CNN
F 1 "Ferrite_Bead_Small" V 3479 2900 50  0000 C CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" V 3555 2900 50  0001 C CNN
F 3 "" H 3625 2900 50  0001 C CNN
	1    3625 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	3525 2900 3375 2900
Wire Wire Line
	3375 2900 3375 2850
$Comp
L Device:C_Small C23
U 1 1 5B0514AF
P 3800 3075
F 0 "C23" H 3892 3121 50  0000 L CNN
F 1 "10uF" H 3892 3030 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3800 3075 50  0001 C CNN
F 3 "" H 3800 3075 50  0001 C CNN
	1    3800 3075
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C26
U 1 1 5B0514B6
P 4175 3075
F 0 "C26" H 4267 3121 50  0000 L CNN
F 1 "100nF" H 4267 3030 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4175 3075 50  0001 C CNN
F 3 "" H 4175 3075 50  0001 C CNN
	1    4175 3075
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 2975 3800 2900
Wire Wire Line
	3800 2900 3725 2900
Wire Wire Line
	3800 2900 4175 2900
Wire Wire Line
	4175 2900 4175 2975
Connection ~ 3800 2900
Wire Wire Line
	4500 2900 4175 2900
Connection ~ 4175 2900
Wire Wire Line
	3800 3175 3800 3225
Wire Wire Line
	3800 3225 4175 3225
Wire Wire Line
	4175 3175 4175 3225
Connection ~ 4175 3225
Wire Wire Line
	4175 3225 4500 3225
$Comp
L Device:C_Small C24
U 1 1 5B0514CB
P 4000 3475
F 0 "C24" H 4000 3550 50  0000 L CNN
F 1 "100nF" H 3700 3475 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4000 3475 50  0001 C CNN
F 3 "" H 4000 3475 50  0001 C CNN
	1    4000 3475
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3375 4000 3350
Wire Wire Line
	4000 3350 4500 3350
Wire Wire Line
	4500 4150 4200 4150
Wire Wire Line
	3375 4150 3375 3875
$Comp
L Device:C_Small C22
U 1 1 5B0514D6
P 3375 3775
F 0 "C22" H 3467 3821 50  0000 L CNN
F 1 "10uF" H 3467 3730 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3375 3775 50  0001 C CNN
F 3 "" H 3375 3775 50  0001 C CNN
	1    3375 3775
	1    0    0    -1  
$EndComp
Wire Wire Line
	3375 3675 3375 3600
Connection ~ 3375 2900
Wire Wire Line
	4000 3350 3375 3350
Connection ~ 4000 3350
Connection ~ 3375 3350
Wire Wire Line
	3375 3350 3375 2900
Wire Wire Line
	4000 3575 4000 4150
Connection ~ 4000 4150
Wire Wire Line
	4000 4150 3900 4150
$Comp
L power:GND #PWR047
U 1 1 5B0514E6
P 3900 4225
F 0 "#PWR047" H 3900 3975 50  0001 C CNN
F 1 "GND" H 3905 4052 50  0000 C CNN
F 2 "" H 3900 4225 50  0001 C CNN
F 3 "" H 3900 4225 50  0001 C CNN
	1    3900 4225
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4225 3900 4150
Connection ~ 3900 4150
Wire Wire Line
	3900 4150 3375 4150
$Comp
L Device:C_Small C25
U 1 1 5B0514F0
P 4100 3750
F 0 "C25" H 4192 3796 50  0000 L CNN
F 1 "100nF" H 4192 3705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4100 3750 50  0001 C CNN
F 3 "" H 4100 3750 50  0001 C CNN
	1    4100 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3650 4100 3600
Wire Wire Line
	4100 3600 4500 3600
Wire Wire Line
	4100 3600 3375 3600
Connection ~ 4100 3600
Connection ~ 3375 3600
Wire Wire Line
	3375 3600 3375 3350
Wire Wire Line
	4100 3850 4100 4150
Connection ~ 4100 4150
Wire Wire Line
	4100 4150 4000 4150
$Comp
L Device:C_Small C27
U 1 1 5B051501
P 4200 3975
F 0 "C27" H 4292 4021 50  0000 L CNN
F 1 "1uF" H 4292 3930 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4200 3975 50  0001 C CNN
F 3 "" H 4200 3975 50  0001 C CNN
	1    4200 3975
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3875 4200 3850
Wire Wire Line
	4200 3850 4500 3850
Wire Wire Line
	4200 4075 4200 4150
Connection ~ 4200 4150
Wire Wire Line
	4200 4150 4100 4150
Text HLabel 4500 2900 2    50   Input ~ 0
VDDANA
Text HLabel 4500 3225 2    50   Input ~ 0
GNDANA
Text HLabel 4500 3350 2    50   Input ~ 0
VDDIO
Text HLabel 4500 3850 2    50   Input ~ 0
VDDCORE
Text HLabel 4500 4150 2    50   Input ~ 0
GND
Text HLabel 4500 3600 2    50   Input ~ 0
VDDIN
$EndSCHEMATC
