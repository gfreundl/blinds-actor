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
L ArduinoProMini:arduino-pro-mini U?
U 1 1 61D42761
P 4080 3350
F 0 "U?" H 4080 4337 55  0001 C CNN
F 1 "Arduino Pro Mini" H 4100 4380 55  0000 C CNN
F 2 "" H 3980 4200 55  0001 C CNN
F 3 "" H 3980 4200 55  0001 C CNN
	1    4080 3350
	1    0    0    -1  
$EndComp
$Comp
L CC1101_gfe:CC1101 U?
U 1 1 61D44387
P 6880 2790
F 0 "U?" H 7305 3031 79  0001 C CNN
F 1 "CC1101" H 7305 2895 79  0000 C CNN
F 2 "" H 6830 2790 79  0001 C CNN
F 3 "" H 6830 2790 79  0001 C CNN
	1    6880 2790
	1    0    0    -1  
$EndComp
$Comp
L DRV8838:DRV8838 U?
U 1 1 61D469B3
P 5370 3200
F 0 "U?" H 5495 3625 50  0001 C CNN
F 1 "DRV8838" H 5495 3534 50  0000 C CNN
F 2 "" H 5320 3200 50  0001 C CNN
F 3 "" H 5320 3200 50  0001 C CNN
	1    5370 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 61D486EF
P 4730 2480
F 0 "R1" H 4789 2526 50  0000 L CNN
F 1 "10R" H 4789 2435 50  0000 L CNN
F 2 "" H 4730 2480 50  0001 C CNN
F 3 "~" H 4730 2480 50  0001 C CNN
	1    4730 2480
	1    0    0    -1  
$EndComp
Wire Wire Line
	5120 3100 5050 3100
Wire Wire Line
	5050 3100 5050 3200
Wire Wire Line
	5050 3200 5120 3200
$Comp
L Device:Battery BT?
U 1 1 61D4C044
P 2540 3110
F 0 "BT?" H 2648 3156 50  0000 L CNN
F 1 "Batteries 2xAA" H 2648 3065 50  0000 L CNN
F 2 "" V 2540 3170 50  0001 C CNN
F 3 "~" V 2540 3170 50  0001 C CNN
	1    2540 3110
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 61D4D100
P 2540 2180
F 0 "#PWR?" H 2540 2030 50  0001 C CNN
F 1 "+BATT" H 2555 2353 50  0000 C CNN
F 2 "" H 2540 2180 50  0001 C CNN
F 3 "" H 2540 2180 50  0001 C CNN
	1    2540 2180
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 61D4DE67
P 3530 2180
F 0 "#PWR?" H 3530 2030 50  0001 C CNN
F 1 "+BATT" H 3545 2353 50  0000 C CNN
F 2 "" H 3530 2180 50  0001 C CNN
F 3 "" H 3530 2180 50  0001 C CNN
	1    3530 2180
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 61D4EFED
P 6620 2190
F 0 "#PWR?" H 6620 2040 50  0001 C CNN
F 1 "+BATT" H 6635 2363 50  0000 C CNN
F 2 "" H 6620 2190 50  0001 C CNN
F 3 "" H 6620 2190 50  0001 C CNN
	1    6620 2190
	1    0    0    -1  
$EndComp
Wire Wire Line
	2540 2910 2540 2180
Wire Wire Line
	3730 2700 3530 2700
Wire Wire Line
	3530 2700 3530 2180
Wire Wire Line
	6780 2840 6610 2840
Wire Wire Line
	6620 2190 6610 2840
$Comp
L power:GND #PWR?
U 1 1 61D54E2A
P 2540 4310
F 0 "#PWR?" H 2540 4060 50  0001 C CNN
F 1 "GND" H 2545 4137 50  0000 C CNN
F 2 "" H 2540 4310 50  0001 C CNN
F 3 "" H 2540 4310 50  0001 C CNN
	1    2540 4310
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61D5735A
P 3640 4310
F 0 "#PWR?" H 3640 4060 50  0001 C CNN
F 1 "GND" H 3645 4137 50  0000 C CNN
F 2 "" H 3640 4310 50  0001 C CNN
F 3 "" H 3640 4310 50  0001 C CNN
	1    3640 4310
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61D58285
P 4930 4310
F 0 "#PWR?" H 4930 4060 50  0001 C CNN
F 1 "GND" H 4935 4137 50  0000 C CNN
F 2 "" H 4930 4310 50  0001 C CNN
F 3 "" H 4930 4310 50  0001 C CNN
	1    4930 4310
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61D59447
P 6420 4300
F 0 "#PWR?" H 6420 4050 50  0001 C CNN
F 1 "GND" H 6425 4127 50  0000 C CNN
F 2 "" H 6420 4300 50  0001 C CNN
F 3 "" H 6420 4300 50  0001 C CNN
	1    6420 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6780 2940 6420 2940
Wire Wire Line
	6420 2940 6420 4300
Wire Wire Line
	5120 3000 4930 3000
Wire Wire Line
	4930 3000 4930 4310
Wire Wire Line
	3730 4150 3640 4150
Wire Wire Line
	3640 4150 3640 4310
Wire Wire Line
	2540 3310 2540 4310
$Comp
L power:+BATT #PWR?
U 1 1 61D5A60F
P 4730 2180
F 0 "#PWR?" H 4730 2030 50  0001 C CNN
F 1 "+BATT" H 4745 2353 50  0000 C CNN
F 2 "" H 4730 2180 50  0001 C CNN
F 3 "" H 4730 2180 50  0001 C CNN
	1    4730 2180
	1    0    0    -1  
$EndComp
Wire Wire Line
	4730 2380 4730 2180
Wire Wire Line
	4430 2600 4730 2600
Wire Wire Line
	4730 2600 4730 2580
$Comp
L Motor:Motor_DC M?
U 1 1 61D5D805
P 5740 5310
F 0 "M?" H 5898 5306 50  0001 L CNN
F 1 "Motor_DC" H 5180 5220 50  0000 L CNN
F 2 "" H 5740 5220 50  0001 C CNN
F 3 "~" H 5740 5220 50  0001 C CNN
	1    5740 5310
	1    0    0    -1  
$EndComp
Wire Wire Line
	5870 3200 6120 3200
Wire Wire Line
	6120 3200 6120 5680
Wire Wire Line
	6120 5680 5740 5680
Wire Wire Line
	5740 5680 5740 5610
Wire Wire Line
	5870 3300 6030 3300
Wire Wire Line
	6030 3300 6030 5030
Wire Wire Line
	6030 5030 5740 5030
Wire Wire Line
	5740 5030 5740 5110
Wire Wire Line
	4730 2600 4730 3650
Wire Wire Line
	4730 3650 5940 3650
Connection ~ 4730 2600
Wire Wire Line
	5940 3650 5940 3400
Wire Wire Line
	5940 3400 5870 3400
Wire Wire Line
	6780 3440 6320 3440
Wire Wire Line
	6320 3440 6320 2690
Wire Wire Line
	6320 2690 4520 2690
Wire Wire Line
	4520 2690 4520 3050
Wire Wire Line
	4520 3050 4430 3050
Wire Wire Line
	4430 3450 4830 3450
Wire Wire Line
	4830 3450 4830 3200
Wire Wire Line
	4830 3200 5050 3200
Connection ~ 5050 3200
Wire Wire Line
	4430 3550 5010 3550
Wire Wire Line
	5010 3550 5010 3300
Wire Wire Line
	5010 3300 5120 3300
Wire Wire Line
	5120 3400 5120 3750
Wire Wire Line
	5120 3750 4430 3750
Wire Wire Line
	6780 3040 6210 3040
Wire Wire Line
	6210 3040 6210 3950
Wire Wire Line
	6210 3950 4430 3950
Wire Wire Line
	4430 3850 6730 3850
Wire Wire Line
	6730 3850 6730 3540
Wire Wire Line
	6730 3540 6780 3540
Wire Wire Line
	6780 3240 6510 3240
Wire Wire Line
	6510 3240 6510 4050
Wire Wire Line
	6510 4050 4430 4050
Wire Wire Line
	6780 3140 6620 3140
Wire Wire Line
	6620 3140 6620 4150
Wire Wire Line
	6620 4150 4430 4150
$Comp
L Device:Antenna AE?
U 1 1 61D67629
P 8240 3040
F 0 "AE?" H 8320 3029 50  0001 L CNN
F 1 "Antenna" H 8320 2983 50  0000 L CNN
F 2 "" H 8240 3040 50  0001 C CNN
F 3 "~" H 8240 3040 50  0001 C CNN
	1    8240 3040
	1    0    0    -1  
$EndComp
Wire Wire Line
	7830 3240 8240 3240
$EndSCHEMATC
