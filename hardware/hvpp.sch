EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "cheap healer by chip killer"
Date "2021-05-20"
Rev "1.4"
Comp "test firmware chiphealer.hex"
Comment1 "by ozforester"
Comment2 "CC-BY-SA 4.0 license"
Comment3 "tested with dip28 megas ( 8/328)"
Comment4 "mega8 && HD44780 16x2 LCD w/ PCF8574A twi"
$EndDescr
$Comp
L power:+5V #PWR07
U 1 1 605BE4A7
P 2850 1900
F 0 "#PWR07" H 2850 1750 50  0001 C CNN
F 1 "+5V" H 2865 2073 50  0000 C CNN
F 2 "" H 2850 1900 50  0001 C CNN
F 3 "" H 2850 1900 50  0001 C CNN
	1    2850 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 2100 2850 2100
Connection ~ 2850 2100
$Comp
L power:GND #PWR08
U 1 1 605C14C7
P 2950 5150
F 0 "#PWR08" H 2950 4900 50  0001 C CNN
F 1 "GND" H 2955 4977 50  0000 C CNN
F 2 "" H 2950 5150 50  0001 C CNN
F 3 "" H 2950 5150 50  0001 C CNN
	1    2950 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 5050 2950 5050
Connection ~ 2950 5050
Wire Wire Line
	2950 5050 2950 5150
Text Label 4025 4775 0    50   ~ 0
RST5
Wire Wire Line
	2850 4950 2850 5050
Wire Wire Line
	2850 2100 2850 2150
Wire Wire Line
	2950 4950 2950 5050
Wire Wire Line
	2950 2150 2950 2100
$Comp
L Device:R R6
U 1 1 6064C904
P 3175 6700
F 0 "R6" H 2968 6700 50  0000 C CNN
F 1 "22k" H 2975 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3105 6700 50  0001 C CNN
F 3 "~" H 3175 6700 50  0001 C CNN
	1    3175 6700
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW4
U 1 1 6064FA0F
P 3175 6350
F 0 "SW4" V 3125 6500 50  0000 L CNN
F 1 "Esc" V 3225 6500 50  0000 L CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 3175 6550 50  0001 C CNN
F 3 "~" H 3175 6550 50  0001 C CNN
	1    3175 6350
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 6066D488
P 3175 6850
F 0 "#PWR09" H 3175 6600 50  0001 C CNN
F 1 "GND" H 3180 6677 50  0000 C CNN
F 2 "" H 3175 6850 50  0001 C CNN
F 3 "" H 3175 6850 50  0001 C CNN
	1    3175 6850
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 606140B6
P 6025 6825
F 0 "#PWR019" H 6025 6575 50  0001 C CNN
F 1 "GND" H 6030 6652 50  0000 C CNN
F 2 "" H 6025 6825 50  0001 C CNN
F 3 "" H 6025 6825 50  0001 C CNN
	1    6025 6825
	-1   0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM7805_TO220 U2
U 1 1 60625A82
P 4975 6475
F 0 "U2" H 4975 6775 50  0000 C CNN
F 1 "LM7805_TO220" H 4975 6675 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4975 6700 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 4975 6425 50  0001 C CNN
	1    4975 6475
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 60626C77
P 4975 6775
F 0 "#PWR027" H 4975 6525 50  0001 C CNN
F 1 "GND" H 4980 6602 50  0000 C CNN
F 2 "" H 4975 6775 50  0001 C CNN
F 3 "" H 4975 6775 50  0001 C CNN
	1    4975 6775
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR031
U 1 1 606271F4
P 5625 6475
F 0 "#PWR031" H 5625 6325 50  0001 C CNN
F 1 "+12V" H 5640 6648 50  0000 C CNN
F 2 "" H 5625 6475 50  0001 C CNN
F 3 "" H 5625 6475 50  0001 C CNN
	1    5625 6475
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR023
U 1 1 60627E21
P 4275 6475
F 0 "#PWR023" H 4275 6325 50  0001 C CNN
F 1 "+5V" H 4290 6648 50  0000 C CNN
F 2 "" H 4275 6475 50  0001 C CNN
F 3 "" H 4275 6475 50  0001 C CNN
	1    4275 6475
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5275 6475 5425 6475
$Comp
L Device:C C3
U 1 1 6063C164
P 4475 6625
F 0 "C3" H 4625 6675 50  0000 L CNN
F 1 "0.1u" H 4625 6575 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4513 6475 50  0001 C CNN
F 3 "~" H 4475 6625 50  0001 C CNN
	1    4475 6625
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4475 6475 4275 6475
$Comp
L Device:C C2
U 1 1 6063C456
P 5425 6625
F 0 "C2" H 5125 6675 50  0000 L CNN
F 1 "0.1u" H 5125 6575 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5463 6475 50  0001 C CNN
F 3 "~" H 5425 6625 50  0001 C CNN
	1    5425 6625
	-1   0    0    -1  
$EndComp
Connection ~ 5425 6475
Wire Wire Line
	5425 6475 5625 6475
$Comp
L power:GND #PWR024
U 1 1 6063C7FC
P 5425 6775
F 0 "#PWR024" H 5425 6525 50  0001 C CNN
F 1 "GND" H 5430 6602 50  0000 C CNN
F 2 "" H 5425 6775 50  0001 C CNN
F 3 "" H 5425 6775 50  0001 C CNN
	1    5425 6775
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 6063CA0E
P 4475 6775
F 0 "#PWR028" H 4475 6525 50  0001 C CNN
F 1 "GND" H 4480 6602 50  0000 C CNN
F 2 "" H 4475 6775 50  0001 C CNN
F 3 "" H 4475 6775 50  0001 C CNN
	1    4475 6775
	-1   0    0    -1  
$EndComp
NoConn ~ 6300 3050
$Comp
L power:GND #PWR02
U 1 1 60619090
P 1400 6850
F 0 "#PWR02" H 1400 6600 50  0001 C CNN
F 1 "GND" H 1405 6677 50  0000 C CNN
F 2 "" H 1400 6850 50  0001 C CNN
F 3 "" H 1400 6850 50  0001 C CNN
	1    1400 6850
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60613D8C
P 1400 6700
F 0 "R2" H 1250 6700 50  0000 C CNN
F 1 "10k" H 1250 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1330 6700 50  0001 C CNN
F 3 "~" H 1400 6700 50  0001 C CNN
	1    1400 6700
	1    0    0    1   
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 6061FE34
P 1400 5725
F 0 "#PWR01" H 1400 5575 50  0001 C CNN
F 1 "+5V" H 1415 5898 50  0000 C CNN
F 2 "" H 1400 5725 50  0001 C CNN
F 3 "" H 1400 5725 50  0001 C CNN
	1    1400 5725
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6061DBAC
P 1400 5875
F 0 "R1" H 1250 5775 50  0000 C CNN
F 1 "10k" H 1250 5875 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1330 5875 50  0001 C CNN
F 3 "~" H 1400 5875 50  0001 C CNN
	1    1400 5875
	1    0    0    1   
$EndComp
$Comp
L Device:R R5
U 1 1 6069D655
P 2725 6700
F 0 "R5" H 2518 6700 50  0000 C CNN
F 1 "10k" H 2525 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2655 6700 50  0001 C CNN
F 3 "~" H 2725 6700 50  0001 C CNN
	1    2725 6700
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 6069D65B
P 2725 6350
F 0 "SW3" V 2675 6500 50  0000 L CNN
F 1 "<" V 2775 6500 50  0000 L CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 2725 6550 50  0001 C CNN
F 3 "~" H 2725 6550 50  0001 C CNN
	1    2725 6350
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 6069D661
P 2725 6850
F 0 "#PWR06" H 2725 6600 50  0001 C CNN
F 1 "GND" H 2730 6677 50  0000 C CNN
F 2 "" H 2725 6850 50  0001 C CNN
F 3 "" H 2725 6850 50  0001 C CNN
	1    2725 6850
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 606A1A7E
P 2275 6700
F 0 "R4" H 2068 6700 50  0000 C CNN
F 1 "2.7k" H 2075 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2205 6700 50  0001 C CNN
F 3 "~" H 2275 6700 50  0001 C CNN
	1    2275 6700
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 606A1A84
P 2275 6350
F 0 "SW2" V 2225 6500 50  0000 L CNN
F 1 ">" V 2325 6500 50  0000 L CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 2275 6550 50  0001 C CNN
F 3 "~" H 2275 6550 50  0001 C CNN
	1    2275 6350
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 606A1A8A
P 2275 6850
F 0 "#PWR05" H 2275 6600 50  0001 C CNN
F 1 "GND" H 2280 6677 50  0000 C CNN
F 2 "" H 2275 6850 50  0001 C CNN
F 3 "" H 2275 6850 50  0001 C CNN
	1    2275 6850
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 606A5E67
P 1825 6700
F 0 "R3" H 1618 6700 50  0000 C CNN
F 1 "560" H 1625 6800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1755 6700 50  0001 C CNN
F 3 "~" H 1825 6700 50  0001 C CNN
	1    1825 6700
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 606A5E6D
P 1825 6350
F 0 "SW1" V 1775 6500 50  0000 L CNN
F 1 "Ent" V 1875 6500 50  0000 L CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 1825 6550 50  0001 C CNN
F 3 "~" H 1825 6550 50  0001 C CNN
	1    1825 6350
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 606A5E73
P 1825 6850
F 0 "#PWR03" H 1825 6600 50  0001 C CNN
F 1 "GND" H 1830 6677 50  0000 C CNN
F 2 "" H 1825 6850 50  0001 C CNN
F 3 "" H 1825 6850 50  0001 C CNN
	1    1825 6850
	-1   0    0    -1  
$EndComp
Connection ~ 3175 6150
Wire Wire Line
	3175 6150 2725 6150
Connection ~ 2725 6150
Wire Wire Line
	2725 6150 2275 6150
Connection ~ 2275 6150
Wire Wire Line
	2275 6150 1825 6150
NoConn ~ 6300 2850
Wire Wire Line
	2850 1900 2850 2100
Connection ~ 4475 6475
$Comp
L power:PWR_FLAG #FLG01
U 1 1 60599CFF
P 6025 6775
F 0 "#FLG01" H 6025 6850 50  0001 C CNN
F 1 "PWR_FLAG" V 6025 6925 50  0001 L CNN
F 2 "" H 6025 6775 50  0001 C CNN
F 3 "~" H 6025 6775 50  0001 C CNN
	1    6025 6775
	1    0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 60598FCA
P 5425 6475
F 0 "#FLG03" H 5425 6550 50  0001 C CNN
F 1 "PWR_FLAG" V 5425 6625 50  0001 L CNN
F 2 "" H 5425 6475 50  0001 C CNN
F 3 "~" H 5425 6475 50  0001 C CNN
	1    5425 6475
	0    1    -1   0   
$EndComp
Wire Wire Line
	4675 6475 4475 6475
Wire Wire Line
	6025 6675 6025 6775
Connection ~ 6025 6775
Wire Wire Line
	6025 6775 6025 6825
$Comp
L power:+12V #PWR018
U 1 1 6060F975
P 6025 6475
F 0 "#PWR018" H 6025 6325 50  0001 C CNN
F 1 "+12V" H 6040 6648 50  0000 C CNN
F 2 "" H 6025 6475 50  0001 C CNN
F 3 "" H 6025 6475 50  0001 C CNN
	1    6025 6475
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 6058AC6E
P 7000 5150
F 0 "#PWR020" H 7000 4900 50  0001 C CNN
F 1 "GND" H 7005 4977 50  0000 C CNN
F 2 "" H 7000 5150 50  0001 C CNN
F 3 "" H 7000 5150 50  0001 C CNN
	1    7000 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4950 7000 5000
Wire Wire Line
	6900 4950 6900 5000
Wire Wire Line
	6900 5000 7000 5000
Connection ~ 7000 5000
Wire Wire Line
	7000 5000 7000 5150
NoConn ~ 7500 3450
NoConn ~ 7500 3550
NoConn ~ 7500 3650
NoConn ~ 7500 3850
$Comp
L Device:R R21
U 1 1 605D605A
P 7950 2850
F 0 "R21" V 7900 3050 50  0000 C CNN
F 1 "1k" V 7900 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 2850 50  0001 C CNN
F 3 "~" H 7950 2850 50  0001 C CNN
	1    7950 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R R32
U 1 1 605D9CF9
P 7950 4550
F 0 "R32" V 7900 4750 50  0000 C CNN
F 1 "1k" V 7900 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 4550 50  0001 C CNN
F 3 "~" H 7950 4550 50  0001 C CNN
	1    7950 4550
	0    1    1    0   
$EndComp
$Comp
L Device:R R31
U 1 1 605D9766
P 7950 4450
F 0 "R31" V 7900 4650 50  0000 C CNN
F 1 "1k" V 7900 4850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 4450 50  0001 C CNN
F 3 "~" H 7950 4450 50  0001 C CNN
	1    7950 4450
	0    1    1    0   
$EndComp
$Comp
L Device:R R30
U 1 1 605D91D1
P 7950 4350
F 0 "R30" V 7900 4550 50  0000 C CNN
F 1 "1k" V 7900 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 4350 50  0001 C CNN
F 3 "~" H 7950 4350 50  0001 C CNN
	1    7950 4350
	0    1    1    0   
$EndComp
$Comp
L Device:R R29
U 1 1 605D8C83
P 7950 4250
F 0 "R29" V 7900 4450 50  0000 C CNN
F 1 "1k" V 7900 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 4250 50  0001 C CNN
F 3 "~" H 7950 4250 50  0001 C CNN
	1    7950 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R R28
U 1 1 605D872D
P 7950 4150
F 0 "R28" V 7900 4350 50  0000 C CNN
F 1 "1k" V 7900 4550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 4150 50  0001 C CNN
F 3 "~" H 7950 4150 50  0001 C CNN
	1    7950 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	7800 4050 7500 4050
$Comp
L Device:R R27
U 1 1 605D81C0
P 7950 4050
F 0 "R27" V 7900 4250 50  0000 C CNN
F 1 "1k" V 7900 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 4050 50  0001 C CNN
F 3 "~" H 7950 4050 50  0001 C CNN
	1    7950 4050
	0    1    1    0   
$EndComp
$Comp
L Device:R R26
U 1 1 605D7B08
P 7950 3950
F 0 "R26" V 7900 4150 50  0000 C CNN
F 1 "1k" V 7900 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 3950 50  0001 C CNN
F 3 "~" H 7950 3950 50  0001 C CNN
	1    7950 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	7800 3350 7500 3350
$Comp
L Device:R R25
U 1 1 605D7347
P 7950 3350
F 0 "R25" V 7900 3550 50  0000 C CNN
F 1 "1k" V 7900 3750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 3350 50  0001 C CNN
F 3 "~" H 7950 3350 50  0001 C CNN
	1    7950 3350
	0    1    1    0   
$EndComp
$Comp
L Device:R R24
U 1 1 605D6EB5
P 7950 3250
F 0 "R24" V 7900 3450 50  0000 C CNN
F 1 "1k" V 7900 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 3250 50  0001 C CNN
F 3 "~" H 7950 3250 50  0001 C CNN
	1    7950 3250
	0    1    1    0   
$EndComp
$Comp
L Device:R R23
U 1 1 605D6970
P 7950 3150
F 0 "R23" V 7900 3350 50  0000 C CNN
F 1 "1k" V 7900 3550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 3150 50  0001 C CNN
F 3 "~" H 7950 3150 50  0001 C CNN
	1    7950 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R R22
U 1 1 605D64AA
P 7950 2950
F 0 "R22" V 7900 3150 50  0000 C CNN
F 1 "1k" V 7900 3350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 2950 50  0001 C CNN
F 3 "~" H 7950 2950 50  0001 C CNN
	1    7950 2950
	0    1    1    0   
$EndComp
$Comp
L Device:R R20
U 1 1 605D5C5C
P 7950 2750
F 0 "R20" V 7900 2950 50  0000 C CNN
F 1 "1k" V 7900 3150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 2750 50  0001 C CNN
F 3 "~" H 7950 2750 50  0001 C CNN
	1    7950 2750
	0    1    1    0   
$EndComp
$Comp
L Device:R R19
U 1 1 605D56E3
P 7950 2650
F 0 "R19" V 7900 2850 50  0000 C CNN
F 1 "1k" V 7900 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 2650 50  0001 C CNN
F 3 "~" H 7950 2650 50  0001 C CNN
	1    7950 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R R18
U 1 1 605D4DE2
P 7950 2550
F 0 "R18" V 7900 2750 50  0000 C CNN
F 1 "1k" V 7900 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 2550 50  0001 C CNN
F 3 "~" H 7950 2550 50  0001 C CNN
	1    7950 2550
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 605D4687
P 7950 2450
F 0 "R17" V 7900 2650 50  0000 C CNN
F 1 "1k" V 7900 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7880 2450 50  0001 C CNN
F 3 "~" H 7950 2450 50  0001 C CNN
	1    7950 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 3250 7800 3250
Wire Wire Line
	7500 3150 7800 3150
Wire Wire Line
	7500 2950 7800 2950
Wire Wire Line
	7500 2750 7800 2750
Wire Wire Line
	7500 2550 7800 2550
Wire Wire Line
	7500 3950 7800 3950
Wire Wire Line
	7500 4550 7800 4550
Wire Wire Line
	7500 4450 7800 4450
Wire Wire Line
	7500 4350 7800 4350
Wire Wire Line
	7500 4250 7800 4250
Wire Wire Line
	7500 4150 7800 4150
Wire Wire Line
	8750 4050 8100 4050
Wire Wire Line
	8750 3350 8100 3350
Wire Wire Line
	8100 2450 8750 2450
Wire Wire Line
	8100 2550 8750 2550
Wire Wire Line
	8100 2650 8750 2650
Wire Wire Line
	8100 2750 8750 2750
Wire Wire Line
	8100 2950 8750 2950
Wire Wire Line
	8100 3150 8750 3150
Wire Wire Line
	8100 3250 8750 3250
Wire Wire Line
	8100 3950 8750 3950
Wire Wire Line
	8100 4150 8750 4150
Wire Wire Line
	8100 4250 8750 4250
Wire Wire Line
	8100 4350 8750 4350
Wire Wire Line
	8100 4450 8750 4450
Wire Wire Line
	8100 4550 8750 4550
Wire Wire Line
	8100 2850 8750 2850
Text Label 8450 3250 0    50   ~ 0
DATA7
Text Label 8450 3150 0    50   ~ 0
DATA6
Text Label 8450 2950 0    50   ~ 0
DATA5
Text Label 8450 2850 0    50   ~ 0
DATA4
Text Label 8450 2750 0    50   ~ 0
DATA3
Text Label 8450 2650 0    50   ~ 0
DATA2
Text Label 8450 2550 0    50   ~ 0
DATA1
Text Label 8450 2450 0    50   ~ 0
DATA0
Text Label 8450 3350 0    50   ~ 0
BS2
Text Label 8450 4550 0    50   ~ 0
PAGEL
Text Label 8450 4450 0    50   ~ 0
XA1
Text Label 8450 4350 0    50   ~ 0
XA0
Text Label 8450 4250 0    50   ~ 0
BS1
Text Label 8450 4150 0    50   ~ 0
WR
Text Label 8450 4050 0    50   ~ 0
OE
Text Label 8450 3950 0    50   ~ 0
RDY
Wire Wire Line
	5050 3875 5050 4025
$Comp
L power:GND #PWR013
U 1 1 605AC5FA
P 4700 5200
F 0 "#PWR013" H 4700 4950 50  0001 C CNN
F 1 "GND" H 4705 5027 50  0000 C CNN
F 2 "" H 4700 5200 50  0001 C CNN
F 3 "" H 4700 5200 50  0001 C CNN
	1    4700 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4975 5050 5200
$Comp
L Device:R R10
U 1 1 605AD8C7
P 4500 4775
F 0 "R10" V 4293 4775 50  0000 C CNN
F 1 "10k" V 4384 4775 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4430 4775 50  0001 C CNN
F 3 "~" H 4500 4775 50  0001 C CNN
	1    4500 4775
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 605AF50D
P 4275 4475
F 0 "R8" H 4475 4425 50  0000 R CNN
F 1 "1k" H 4500 4525 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4205 4475 50  0001 C CNN
F 3 "~" H 4275 4475 50  0001 C CNN
	1    4275 4475
	-1   0    0    1   
$EndComp
Wire Wire Line
	4275 4625 4275 4775
Wire Wire Line
	4275 4275 4275 4325
$Comp
L power:+5V #PWR011
U 1 1 605B237B
P 4275 3875
F 0 "#PWR011" H 4275 3725 50  0001 C CNN
F 1 "+5V" H 4290 4048 50  0000 C CNN
F 2 "" H 4275 3875 50  0001 C CNN
F 3 "" H 4275 3875 50  0001 C CNN
	1    4275 3875
	1    0    0    -1  
$EndComp
Wire Wire Line
	4275 3875 4275 3975
$Comp
L Device:R R15
U 1 1 605B4B3C
P 5050 4175
F 0 "R15" H 4980 4129 50  0000 R CNN
F 1 "1k" H 4980 4220 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4980 4175 50  0001 C CNN
F 3 "~" H 5050 4175 50  0001 C CNN
	1    5050 4175
	-1   0    0    1   
$EndComp
Wire Wire Line
	5950 2650 5650 2650
Text Label 5725 2650 0    50   ~ 0
XTAL1
Text Label 6000 2450 0    50   ~ 0
RST12
Entry Wire Line
	8750 3950 8850 4050
Entry Wire Line
	8750 3250 8850 3350
Entry Wire Line
	8750 3150 8850 3250
Entry Wire Line
	8750 2950 8850 3050
Entry Wire Line
	8750 2850 8850 2950
Entry Wire Line
	8750 2750 8850 2850
Entry Wire Line
	8750 2650 8850 2750
Entry Wire Line
	8750 2550 8850 2650
Entry Wire Line
	8750 3350 8850 3450
Entry Wire Line
	8750 4550 8850 4650
Entry Wire Line
	8750 4450 8850 4550
Entry Wire Line
	8750 4350 8850 4450
Entry Wire Line
	8750 4250 8850 4350
Entry Wire Line
	8750 4150 8850 4250
Entry Wire Line
	8750 4050 8850 4150
Entry Wire Line
	8750 2450 8850 2550
Entry Wire Line
	8950 3725 8850 3825
Entry Wire Line
	8950 3625 8850 3725
Entry Wire Line
	5650 5700 5550 5800
Wire Wire Line
	5650 2650 5650 5700
Wire Wire Line
	3450 4350 3800 4350
Wire Wire Line
	3450 4450 3800 4450
Wire Wire Line
	3450 4550 3800 4550
Wire Wire Line
	3800 2450 3450 2450
Wire Wire Line
	3450 2550 3800 2550
Wire Wire Line
	3450 2650 3800 2650
Wire Wire Line
	3450 2750 3800 2750
Wire Wire Line
	3450 2850 3800 2850
Wire Wire Line
	3450 2950 3800 2950
Wire Wire Line
	3450 3850 3800 3850
Wire Wire Line
	3450 3150 3800 3150
Wire Wire Line
	3450 3250 3800 3250
Wire Wire Line
	3450 3350 3800 3350
Wire Wire Line
	3450 3450 3800 3450
Wire Wire Line
	3800 3550 3450 3550
Wire Wire Line
	3450 3650 3800 3650
Wire Wire Line
	3450 3950 3800 3950
Wire Wire Line
	3800 4050 3450 4050
Wire Wire Line
	3800 4150 3450 4150
Wire Wire Line
	3450 4250 3800 4250
Text Label 3500 3450 0    50   ~ 0
BTN
Text Label 3500 4550 0    50   ~ 0
DATA2
Text Label 3500 4450 0    50   ~ 0
DATA3
Text Label 3500 4350 0    50   ~ 0
DATA4
Text Label 1950 2850 0    50   ~ 0
DATA5
Text Label 3500 4150 0    50   ~ 0
DATA6
Text Label 3500 4050 0    50   ~ 0
DATA7
Text Label 3500 3950 0    50   ~ 0
BS2
Text Label 3500 3350 0    50   ~ 0
OE
Text Label 3500 3250 0    50   ~ 0
WR
Text Label 3500 3150 0    50   ~ 0
BS1
Text Label 3500 2950 0    50   ~ 0
XTAL1
Text Label 3500 2850 0    50   ~ 0
XA0
Text Label 3500 2750 0    50   ~ 0
XA1
Text Label 3500 2650 0    50   ~ 0
PAGEL
Text Label 3500 2550 0    50   ~ 0
DATA0
Text Label 3500 2450 0    50   ~ 0
DATA1
Text Label 3500 3650 0    50   ~ 0
SCL
Text Label 3500 3550 0    50   ~ 0
SDA
Text Label 9000 3725 0    50   ~ 0
SDA
Text Label 9000 3625 0    50   ~ 0
SCL
Wire Wire Line
	8950 3725 9250 3725
Connection ~ 9250 3725
Wire Wire Line
	8950 3625 9450 3625
Connection ~ 9450 3625
$Comp
L power:+5V #PWR025
U 1 1 60589D76
P 10050 3125
F 0 "#PWR025" H 10050 2975 50  0001 C CNN
F 1 "+5V" H 10065 3298 50  0000 C CNN
F 2 "" H 10050 3125 50  0001 C CNN
F 3 "" H 10050 3125 50  0001 C CNN
	1    10050 3125
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 3125 10050 3325
Wire Wire Line
	10050 4725 10050 4875
$Comp
L Device:R R33
U 1 1 6057E6B8
P 9250 3275
F 0 "R33" H 9450 3225 50  0000 R CNN
F 1 "2.2k" H 9450 3325 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9180 3275 50  0001 C CNN
F 3 "~" H 9250 3275 50  0001 C CNN
	1    9250 3275
	-1   0    0    1   
$EndComp
$Comp
L Device:R R34
U 1 1 6057F19D
P 9450 3275
F 0 "R34" H 9380 3229 50  0000 R CNN
F 1 "2.2k" H 9380 3320 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9380 3275 50  0001 C CNN
F 3 "~" H 9450 3275 50  0001 C CNN
	1    9450 3275
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR022
U 1 1 6058489F
P 9450 3125
F 0 "#PWR022" H 9450 2975 50  0001 C CNN
F 1 "+5V" H 9465 3298 50  0000 C CNN
F 2 "" H 9450 3125 50  0001 C CNN
F 3 "" H 9450 3125 50  0001 C CNN
	1    9450 3125
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 6058D4A5
P 10050 4875
F 0 "#PWR026" H 10050 4625 50  0001 C CNN
F 1 "GND" H 10055 4702 50  0000 C CNN
F 2 "" H 10050 4875 50  0001 C CNN
F 3 "" H 10050 4875 50  0001 C CNN
	1    10050 4875
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR021
U 1 1 605A7C18
P 9250 3125
F 0 "#PWR021" H 9250 2975 50  0001 C CNN
F 1 "+5V" H 9265 3298 50  0000 C CNN
F 2 "" H 9250 3125 50  0001 C CNN
F 3 "" H 9250 3125 50  0001 C CNN
	1    9250 3125
	1    0    0    -1  
$EndComp
$Comp
L hvpp-rescue:LCD16x2_TWI-LCD-Hvpp-rescue-Hvpp.v1.4-rescue LCD1
U 1 1 6057E349
P 10050 4025
F 0 "LCD1" H 10050 4075 50  0000 L CNN
F 1 "LCD16x2_TWI" H 9850 3975 50  0000 L CNN
F 2 "Hvpp:1602A" H 10050 4025 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCF8574_PCF8574A.pdf" H 10050 4025 50  0001 C CNN
	1    10050 4025
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 3425 9450 3625
Wire Wire Line
	9450 3625 9550 3625
Wire Wire Line
	9250 3425 9250 3725
Wire Wire Line
	9250 3725 9550 3725
Wire Wire Line
	7500 2650 7800 2650
Wire Wire Line
	7500 2850 7800 2850
Wire Wire Line
	7500 2450 7800 2450
Wire Wire Line
	6300 2650 6250 2650
Entry Wire Line
	3800 3650 3900 3750
Entry Wire Line
	3800 3450 3900 3550
Entry Wire Line
	3800 3350 3900 3450
Entry Wire Line
	3800 3250 3900 3350
Entry Wire Line
	3800 3150 3900 3250
Entry Wire Line
	3800 3950 3900 4050
Entry Wire Line
	3800 3850 3900 3950
Entry Wire Line
	3800 3550 3900 3650
Entry Wire Line
	3800 2950 3900 3050
Entry Wire Line
	3800 2850 3900 2950
Entry Wire Line
	3800 2750 3900 2850
Entry Wire Line
	3800 2650 3900 2750
Entry Wire Line
	3800 2550 3900 2650
Entry Wire Line
	3800 4550 3900 4650
Entry Wire Line
	3800 4450 3900 4550
Entry Wire Line
	3800 4350 3900 4450
Entry Wire Line
	3800 4250 3900 4350
Entry Wire Line
	3800 4150 3900 4250
Entry Wire Line
	3800 4050 3900 4150
Entry Wire Line
	3800 2450 3900 2550
Wire Wire Line
	5050 4325 5050 4400
Wire Wire Line
	5050 4400 5425 4400
Connection ~ 5050 4400
Wire Wire Line
	5050 4400 5050 4575
NoConn ~ 6025 6575
$Comp
L Device:R R35
U 1 1 6063B702
P 3750 7025
F 0 "R35" H 3680 6979 50  0000 R CNN
F 1 "1k" H 3680 7070 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3680 7025 50  0001 C CNN
F 3 "~" H 3750 7025 50  0001 C CNN
	1    3750 7025
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D3
U 1 1 6063B709
P 3750 6675
F 0 "D3" V 3789 6558 50  0000 R CNN
F 1 "PWR" V 3698 6558 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3750 6675 50  0001 C CNN
F 3 "~" H 3750 6675 50  0001 C CNN
	1    3750 6675
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3750 6825 3750 6875
$Comp
L power:+5V #PWR029
U 1 1 6063B710
P 3750 6475
F 0 "#PWR029" H 3750 6325 50  0001 C CNN
F 1 "+5V" H 3765 6648 50  0000 C CNN
F 2 "" H 3750 6475 50  0001 C CNN
F 3 "" H 3750 6475 50  0001 C CNN
	1    3750 6475
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 6475 3750 6525
$Comp
L power:GND #PWR030
U 1 1 60657A18
P 3750 7175
F 0 "#PWR030" H 3750 6925 50  0001 C CNN
F 1 "GND" H 3755 7002 50  0000 C CNN
F 2 "" H 3750 7175 50  0001 C CNN
F 3 "" H 3750 7175 50  0001 C CNN
	1    3750 7175
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 605CC267
P 6100 2650
F 0 "R16" V 6000 2700 50  0000 C CNN
F 1 "1k" V 5900 2700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6030 2650 50  0001 C CNN
F 3 "~" H 6100 2650 50  0001 C CNN
	1    6100 2650
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:Housing N1
U 1 1 60665047
P 2900 4400
F 0 "N1" H 3053 4388 50  0001 L CNN
F 1 "Housing" H 3053 4343 50  0001 L CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 2950 4450 50  0001 C CNN
F 3 "~" H 2950 4450 50  0001 C CNN
	1    2900 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 60679D17
P 2125 3375
F 0 "C1" H 2175 3600 50  0000 L CNN
F 1 "0.1u" H 2150 3500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2163 3225 50  0001 C CNN
F 3 "~" H 2125 3375 50  0001 C CNN
	1    2125 3375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 6067BEAF
P 2125 3525
F 0 "#PWR04" H 2125 3275 50  0001 C CNN
F 1 "GND" H 2130 3352 50  0000 C CNN
F 2 "" H 2125 3525 50  0001 C CNN
F 3 "" H 2125 3525 50  0001 C CNN
	1    2125 3525
	1    0    0    -1  
$EndComp
Wire Wire Line
	2125 3050 2250 3050
Entry Wire Line
	4000 4775 3900 4875
Wire Wire Line
	4000 4775 4275 4775
Text Label 3500 3850 0    50   ~ 0
RST5
Wire Wire Line
	1400 6550 1400 6150
Wire Wire Line
	1400 6150 1825 6150
Connection ~ 1825 6150
Wire Wire Line
	1400 6025 1400 6150
Connection ~ 1400 6150
Wire Wire Line
	1900 2650 2250 2650
Wire Wire Line
	2250 2850 1900 2850
Text Label 2000 2650 0    50   ~ 0
VCC
Text Label 3500 4250 0    50   ~ 0
RDY
Wire Wire Line
	2125 3050 2125 3225
Wire Wire Line
	3175 6150 3450 6150
Text Label 3450 6000 0    50   ~ 0
BTN
Wire Wire Line
	6900 2100 6900 2150
Connection ~ 6900 2100
Wire Wire Line
	7000 2100 6900 2100
Wire Wire Line
	7000 2150 7000 2100
Wire Wire Line
	5425 4400 5425 2450
Wire Wire Line
	5425 2450 6300 2450
$Comp
L Device:R R12
U 1 1 6077F1FB
P 4700 5050
F 0 "R12" H 4630 5004 50  0000 R CNN
F 1 "4.7k" H 4630 5095 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4630 5050 50  0001 C CNN
F 3 "~" H 4700 5050 50  0001 C CNN
	1    4700 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4350 4775 4275 4775
Connection ~ 4275 4775
Wire Wire Line
	4650 4775 4700 4775
Wire Wire Line
	4700 4900 4700 4775
Connection ~ 4700 4775
Wire Wire Line
	4700 4775 4750 4775
$Comp
L power:GND #PWR017
U 1 1 607C854C
P 5050 5200
F 0 "#PWR017" H 5050 4950 50  0001 C CNN
F 1 "GND" H 5055 5027 50  0000 C CNN
F 2 "" H 5050 5200 50  0001 C CNN
F 3 "" H 5050 5200 50  0001 C CNN
	1    5050 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 605B0E22
P 4275 4125
F 0 "D2" V 4314 4008 50  0000 R CNN
F 1 "12V" V 4223 4008 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4275 4125 50  0001 C CNN
F 3 "~" H 4275 4125 50  0001 C CNN
	1    4275 4125
	0    -1   -1   0   
$EndComp
Text Label 4025 2875 0    50   ~ 0
VCC
Wire Wire Line
	5025 2050 5025 2175
$Comp
L power:GND #PWR012
U 1 1 60803200
P 4675 3300
F 0 "#PWR012" H 4675 3050 50  0001 C CNN
F 1 "GND" H 4680 3127 50  0000 C CNN
F 2 "" H 4675 3300 50  0001 C CNN
F 3 "" H 4675 3300 50  0001 C CNN
	1    4675 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5025 3075 5025 3300
$Comp
L Device:R R9
U 1 1 60803207
P 4475 2875
F 0 "R9" V 4268 2875 50  0000 C CNN
F 1 "10k" V 4359 2875 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4405 2875 50  0001 C CNN
F 3 "~" H 4475 2875 50  0001 C CNN
	1    4475 2875
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 60803214
P 7475 1800
F 0 "D1" V 7550 1650 50  0000 R CNN
F 1 "5V" V 7450 1650 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7475 1800 50  0001 C CNN
F 3 "~" H 7475 1800 50  0001 C CNN
	1    7475 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R14
U 1 1 60803222
P 5025 2425
F 0 "R14" H 4955 2379 50  0000 R CNN
F 1 "3.3k" H 4955 2470 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4955 2425 50  0001 C CNN
F 3 "~" H 5025 2425 50  0001 C CNN
	1    5025 2425
	-1   0    0    1   
$EndComp
$Comp
L Device:R R11
U 1 1 6080322D
P 4675 3150
F 0 "R11" H 4605 3104 50  0000 R CNN
F 1 "3.3k" H 4605 3195 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4605 3150 50  0001 C CNN
F 3 "~" H 4675 3150 50  0001 C CNN
	1    4675 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4625 2875 4675 2875
Wire Wire Line
	4675 3000 4675 2875
Connection ~ 4675 2875
Wire Wire Line
	4675 2875 4725 2875
$Comp
L power:GND #PWR015
U 1 1 60803239
P 5025 3300
F 0 "#PWR015" H 5025 3050 50  0001 C CNN
F 1 "GND" H 5030 3127 50  0000 C CNN
F 2 "" H 5025 3300 50  0001 C CNN
F 3 "" H 5025 3300 50  0001 C CNN
	1    5025 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5025 2575 5025 2675
Entry Wire Line
	3900 2975 4000 2875
$Comp
L power:+12V #PWR016
U 1 1 6058EA74
P 5050 3875
F 0 "#PWR016" H 5050 3725 50  0001 C CNN
F 1 "+12V" H 5065 4048 50  0000 C CNN
F 2 "" H 5050 3875 50  0001 C CNN
F 3 "" H 5050 3875 50  0001 C CNN
	1    5050 3875
	1    0    0    -1  
$EndComp
Connection ~ 3900 5800
Entry Wire Line
	1800 2750 1900 2650
Entry Wire Line
	3450 5900 3550 5800
Wire Wire Line
	3450 5900 3450 6150
Entry Wire Line
	1800 2950 1900 2850
$Comp
L Device:R R13
U 1 1 6076FA37
P 5025 1900
F 0 "R13" H 5275 1850 50  0000 R CNN
F 1 "10k" H 5250 1950 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4955 1900 50  0001 C CNN
F 3 "~" H 5025 1900 50  0001 C CNN
	1    5025 1900
	-1   0    0    1   
$EndComp
$Comp
L Transistor_FET:IRLML5103 Q3
U 1 1 6077069D
P 5400 1700
F 0 "Q3" V 5300 1900 50  0000 C CNN
F 1 "IRLML5103" V 5200 2025 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5600 1625 50  0001 L CIN
F 3 "https://www.infineon.com/dgdl/irlml6402pbf.pdf?fileId=5546d462533600a401535668d5c2263c" H 5400 1700 50  0001 L CNN
F 4 "X" H 5400 1700 50  0001 C CNN "Spice_Primitive"
F 5 "IRLML5103" H 5400 1700 50  0001 C CNN "Spice_Model"
F 6 "Y" H 5400 1700 50  0001 C CNN "Spice_Netlist_Enabled"
F 7 "Transistor_FET.lib" H 5400 1700 50  0001 C CNN "Spice_Lib_File"
	1    5400 1700
	0    1    -1   0   
$EndComp
Wire Wire Line
	5400 1900 5400 2175
Wire Wire Line
	5400 2175 5025 2175
Connection ~ 5025 2175
Wire Wire Line
	5025 2175 5025 2275
Wire Wire Line
	5600 1600 6900 1600
Wire Wire Line
	5025 1575 5025 1600
Wire Wire Line
	5025 1600 5200 1600
Wire Wire Line
	5025 1600 5025 1750
Connection ~ 5025 1600
$Comp
L power:+5V #PWR014
U 1 1 607F6045
P 5025 1575
F 0 "#PWR014" H 5025 1425 50  0001 C CNN
F 1 "+5V" H 5040 1748 50  0000 C CNN
F 2 "" H 5025 1575 50  0001 C CNN
F 3 "" H 5025 1575 50  0001 C CNN
	1    5025 1575
	1    0    0    -1  
$EndComp
Text Label 5150 4400 0    50   ~ 0
RST12
$Comp
L power:PWR_FLAG #FLG02
U 1 1 6079BBA1
P 6900 2100
F 0 "#FLG02" H 6900 2175 50  0001 C CNN
F 1 "PWR_FLAG" V 6900 2250 50  0001 L CNN
F 2 "" H 6900 2100 50  0001 C CNN
F 3 "~" H 6900 2100 50  0001 C CNN
	1    6900 2100
	0    -1   -1   0   
$EndComp
$Comp
L hvpp-rescue:ATmega8L-8PU-MCU_Microchip_ATmega-Hvpp-rescue-Hvpp.v1.4-rescue S1
U 1 1 605757F4
P 6900 3550
F 0 "S1" H 6800 3700 50  0000 C CNN
F 1 "DIP28" H 6800 3550 50  0000 C CNN
F 2 "Package_DIP:DIP-28_W7.62mm_Socket" H 6900 3550 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2486-8-bit-avr-microcontroller-atmega8_l_datasheet.pdf" H 6900 3550 50  0001 C CNN
	1    6900 3550
	1    0    0    -1  
$EndComp
$Comp
L hvpp-rescue:ATmega8L-8PU-MCU_Microchip_ATmega-Hvpp-rescue-Hvpp.v1.4-rescue U1
U 1 1 60574F66
P 2850 3550
F 0 "U1" H 2800 3850 50  0000 C CNN
F 1 "ATmega8L-8PU" H 2800 3725 50  0000 C CNN
F 2 "Package_DIP:DIP-28_W7.62mm_Socket" H 2850 3550 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2486-8-bit-avr-microcontroller-atmega8_l_datasheet.pdf" H 2850 3550 50  0001 C CNN
	1    2850 3550
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC847C Q1
U 1 1 60898E98
P 4925 2875
F 0 "Q1" H 5115 2921 50  0000 L CNN
F 1 "BC847C" H 5115 2830 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5125 2800 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 4925 2875 50  0001 L CNN
F 4 "X" H 4925 2875 50  0001 C CNN "Spice_Primitive"
F 5 "BC847C" H 4925 2875 50  0001 C CNN "Spice_Model"
F 6 "Y" H 4925 2875 50  0001 C CNN "Spice_Netlist_Enabled"
F 7 "Transistor_BJT.lib" H 4925 2875 50  0001 C CNN "Spice_Lib_File"
	1    4925 2875
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC847 Q2
U 1 1 6059F204
P 4950 4775
F 0 "Q2" H 4875 5050 50  0000 L CNN
F 1 "BC847C" H 4700 4950 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5150 4700 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 4950 4775 50  0001 L CNN
F 4 "X" H 4950 4775 50  0001 C CNN "Spice_Primitive"
F 5 "BC847C" H 4950 4775 50  0001 C CNN "Spice_Model"
F 6 "Y" H 4950 4775 50  0001 C CNN "Spice_Netlist_Enabled"
F 7 "Transistor_BJT.lib" H 4950 4775 50  0001 C CNN "Spice_Lib_File"
	1    4950 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2875 4325 2875
$Comp
L power:GND #PWR0101
U 1 1 6079F902
P 7475 2000
F 0 "#PWR0101" H 7475 1750 50  0001 C CNN
F 1 "GND" H 7625 1925 50  0000 C CNN
F 2 "" H 7475 2000 50  0001 C CNN
F 3 "" H 7475 2000 50  0001 C CNN
	1    7475 2000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack_Switch J1
U 1 1 6064218B
P 6325 6575
F 0 "J1" H 6325 6875 50  0000 R CNN
F 1 "5.5/2.1" H 6450 6800 50  0000 R CNN
F 2 "Hvpp:barreljack" H 6375 6535 50  0001 C CNN
F 3 "~" H 6375 6535 50  0001 C CNN
	1    6325 6575
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6900 1600 6900 2100
$Comp
L Device:R R7
U 1 1 6080320D
P 7150 1600
F 0 "R7" V 7350 1625 50  0000 R CNN
F 1 "1k" V 7250 1650 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7080 1600 50  0001 C CNN
F 3 "~" H 7150 1600 50  0001 C CNN
	1    7150 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7300 1600 7475 1600
Wire Wire Line
	7000 1600 6900 1600
Connection ~ 6900 1600
$Comp
L Mechanical:MountingHole H1
U 1 1 6078C535
P 700 7550
F 0 "H1" H 800 7596 50  0000 L CNN
F 1 "MountingHole" H 800 7505 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_DIN965_Pad" H 700 7550 50  0001 C CNN
F 3 "~" H 700 7550 50  0001 C CNN
	1    700  7550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 607901D8
P 1050 7550
F 0 "H2" H 1150 7596 50  0000 L CNN
F 1 "MountingHole" H 1150 7505 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_DIN965_Pad" H 1050 7550 50  0001 C CNN
F 3 "~" H 1050 7550 50  0001 C CNN
	1    1050 7550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW5
U 1 1 608605E0
P 1975 2450
F 0 "SW5" H 1925 2750 50  0000 L CNN
F 1 "Reset" H 1900 2650 50  0000 L CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 1975 2650 50  0001 C CNN
F 3 "~" H 1975 2650 50  0001 C CNN
	1    1975 2450
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 608612C9
P 1775 2450
F 0 "#PWR010" H 1775 2200 50  0001 C CNN
F 1 "GND" H 1725 2525 50  0001 C CNN
F 2 "" H 1775 2450 50  0001 C CNN
F 3 "" H 1775 2450 50  0001 C CNN
	1    1775 2450
	1    0    0    -1  
$EndComp
Entry Wire Line
	3900 2550 4000 2450
Entry Wire Line
	3900 2650 4000 2550
Entry Wire Line
	3900 2750 4000 2650
Wire Wire Line
	4000 2550 4200 2550
Wire Wire Line
	4200 2550 4200 1325
Wire Wire Line
	4200 1325 3900 1325
Wire Wire Line
	4000 2650 4300 2650
Wire Wire Line
	4300 2650 4300 1225
Wire Wire Line
	4300 1225 3900 1225
Wire Wire Line
	4000 2450 4100 2450
Wire Wire Line
	4100 2450 4100 1425
Wire Wire Line
	4100 1425 3900 1425
$Comp
L power:+5V #PWR0102
U 1 1 608E4914
P 4025 850
F 0 "#PWR0102" H 4025 700 50  0001 C CNN
F 1 "+5V" H 4040 1023 50  0000 C CNN
F 2 "" H 4025 850 50  0001 C CNN
F 3 "" H 4025 850 50  0001 C CNN
	1    4025 850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 608E5787
P 4450 1125
F 0 "#PWR0103" H 4450 875 50  0001 C CNN
F 1 "GND" H 4455 952 50  0000 C CNN
F 2 "" H 4450 1125 50  0001 C CNN
F 3 "" H 4450 1125 50  0001 C CNN
	1    4450 1125
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1025 4025 1025
Wire Wire Line
	4025 1025 4025 850 
Text Label 4100 2300 1    50   ~ 0
XA1
Text Label 4200 2300 1    50   ~ 0
XA0
Text Label 4300 2300 1    50   ~ 0
XTAL1
Text Notes 3450 1450 0    50   ~ 0
mosi
Text Notes 3450 1350 0    50   ~ 0
miso
Text Notes 3500 1250 0    50   ~ 0
sck
Text Notes 3500 1150 0    50   ~ 0
gnd
Text Notes 3500 1050 0    50   ~ 0
vcc
$Comp
L Connector_Generic:Conn_01x06 J2
U 1 1 608B277A
P 3700 1325
F 0 "J2" H 3618 800 50  0000 C CNN
F 1 "USB-ASP" H 3700 900 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 3700 1325 50  0001 C CNN
F 3 "~" H 3700 1325 50  0001 C CNN
	1    3700 1325
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 1125 4450 1125
Text Notes 3525 1550 0    50   ~ 0
rst
Wire Wire Line
	2175 2450 2200 2450
Wire Wire Line
	3900 1525 3900 1625
Wire Wire Line
	3900 1625 2200 1625
Wire Wire Line
	2200 1625 2200 2450
Connection ~ 2200 2450
Wire Wire Line
	2200 2450 2250 2450
Wire Wire Line
	7475 1650 7475 1600
Wire Wire Line
	7475 1950 7475 2000
Text Notes 2650 4000 0    50   ~ 0
Int. RC \n@4MHz\nLF:0x23\nHF:0xD9\nLB:0xFF
Wire Bus Line
	1800 2750 1800 5800
Wire Bus Line
	1800 5800 3900 5800
Wire Bus Line
	3900 5800 8850 5800
Wire Bus Line
	3900 2550 3900 5800
Wire Bus Line
	8850 2550 8850 5800
$EndSCHEMATC
