# Read_RTD_Sensor
## Overview
This repository contains three C++ files for the demonstration of SPI communication initialization measurement and conversion of Resistance Temperature Detector sensor data. Also present are proteus and pdf files which depict the circuitry diagram for the operation of the code. 

## Objective
Read temperature data from two RTD sensors (PT100 and PT1000) via a multiplexer, then digitize it using an external SPI ADC (AD7352) and send the digital readings to the ATmega128 microcontroller.

## Components
The project utilizes the follwoing specific devices:
Temperature sensors (RTD-PT100 & RTD-PT1000)
ADC converter (AD7352)
Multiplexer (ADG1211)
Microcontroller (ATMega128)
Capacitors (microFarads)
Resistors (Ohms)

## Logical Integration
SPI is initialized on MCU (PB3 – MOSI, PB1 – SCLK, PB0 – CS)
Microcontroller selects MUX channel by toggling S1 and S2 lines
Microcontroller sends SPI command to ADC to start operating
It then reads digital data from SDATA_A pin
The raw digital data is converted in MCU to Resistance and to temperature using linear formula
A virtual terminal is used for display of data sent via UART.

## Procedure
#### Hardware
Arduino MCU ATMega128 is used for compliling the code and generating a .hex file extension. The MCU uses pins:
  1. PC0-S1 and PC-S2 to toggle multiplexer channels. 
  2. PB0-CS and PB1-SCLK to activate/deactivate ADC operation and clock setting
  3. PB3-SDATAA for data transfer from ADC

ADC uses pins:
  1. VINA+-D1/D2 to receive analog data from the multiplexer
  2. REFA+ to connect to the reference voltage circuitry

MUX uses pins:
  1. IN1 and IN2 for analog data intake from both RT1 and RT2

#### Software
The hex file generated from the complied Arduino code is uploaded to the the Proteus Source code
After compliling, the virtual terminal is used to view the output
