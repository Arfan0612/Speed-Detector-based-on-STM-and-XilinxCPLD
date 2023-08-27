# Introduction
The goal of the **group** project is to design and build a remote display board that can receive serial data from the STM32-NUCLEO-L476RG and display the detected speed on two 7-segment LED displays. To do this, the receiver system was implemented using a Xilinx CPLD XC2C64A-7VQG44C, which enables flexibility in design testing and updating without requiring any hardware modifications. Thus, greatly reducing the project development time. The CPLD and microcontroller communicate using an industry standard, RS485, ensuring dependable and efficient communication between the components.

![20230418_122524](https://github.com/Arfan0612/Speed-Detector-based-on-STM-and-XilinxCPLD/assets/94776851/de6c9307-1eb1-40f4-bda7-26b5f7d67cb7)

Refer to the report **Speed_Detector_Report_2023.pdf** attached for a better explanation of the system.

# Speed_Detector_STM_Xilinx
This is the main project folder used to run the STM32-NUCLEO-L476RG board. This project was created and built on STM32CubeIDE 1.10.1. The below lists the files added or changed in this project:

## Speed_Detector_STM_Xilinx\Core\Src
In this directory there are 4 files added or changed which are the:
- **main.c** which is the main file that controls the whole operation of the speed detector
- **comparator.c** which controls the comparator algorithm used to determine frequency of input signal using an input signal from a comparator
- **display.c** is a simple algorithm that is used to control the external LCD keypad shield used to display relevant information regarding frequency detected and speed
- **lcd16x2_v2.c** is the code used to manipuluate the LCD keypad shield which was found online
- **Decimal_BCD_converter.c** is the simple algorithm used to convert decimal numbers to BCD before transmission through UART to Xilinx board for decoding

Each files that was listed has their own respective header files in Speed_Detector_STM_Xilinx\Core\Inc

## Speed_Detector_STM_Xilinx\Drivers\CMSIS\DSP\Include
There is one file added in this directory which is **FFT.h** that is used to do FFT on ADC readings for determining the input signal's frequency.

# Rx_final.zip
This zipped folder contains all the schematics used for programming the UART receiver on the Xilinx CPLD XC2C64A-7VQG44C. According to my colleague, Chin Yuen, the folder includes the final schematics used to program a Xilinx CPLD XC2C64A-7VQG44C UART:

## BCDto7Seg
BCD to 7 Segment Decoder, where the file name that does not have a "v" in it is the final schematics used.

## Clk_div_2
A clock divider schematic used to divide the external clock oscillator by 2 to set the CLK for the Xilinx

## EightCounter
Counts from 0 to 8 using 4 D flip-flops 

## TenX
Counts from 0 to 9 using 9 D flip-flops

## SixteenCounter
Counts from 0 to 15 using CB4CE

## StartBD_v3_hold
A start bit detector circuit designed to detect the start bit of the UART transmission from the STM. This schematic follows the Moore machine concept. 


 
