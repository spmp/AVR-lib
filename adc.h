/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file adc.h
 *
 * @brief Header for adc.c, accessig the internal ADC's
 *
 * - Compiler:          gcc-avr
 * - Project:           AVR-lib
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: 2017
 *****************************************************************************/
#pragma once
#include <avr/io.h>

/**
 * @brief Functions to initilise and access the internal 10bit ADC.
 * 
 * The Arduino and MEGA range of uC's have built in 10bit ADC's. These can use
 * a precision 1.1V reference, VCC as reference or an external reference.
 * The ADC can use all 8 pins of PORTA as multiplexed inputs.
 * To use the pin for ADC both DDRA.x and PORTA.x must be cleared.
 * 
 * @resource 10Bit ADC
 * @resource Any of pins A0-A7
 **/

#define VREF_INTERNAL   0
#define VREF_EXTERNAL   1
#define VREF_VCC        2

// Set the voltage of the external reference somewhere with
// #undef EXTERNAL_MILLI_VOLT
// #define EXTERNAL_MILLI_VOLT YourVoltageX1000
#define EXTERNAL_MILLIVOLT 5000

/**
 * @brief Read ADC pin, returning value
 * 
 * @param pin, The ADC pin to be read. 0-7
 * @param vref, The voltage reference. 0 = 1.1, 1 = external, >=2 = Vcc
 * @return uint16, The 10bit value from the init_ADC
 **/ 
uint16_t read_ADC_pin(uint8_t pin, uint8_t vref);

/**
 * @brief Read ADC pin, returning a value in millivolts
 * 
 * @param pin, The ADC pin to be read. 0-7
 * @param vref, The voltage reference. 0 = 1.1, 1 = external, >=2 = Vcc
 * @return uint16, ADC reading in millivolts
 **/ 
uint16_t read_ADC_pin_millivolts(uint8_t pin, uint8_t vref);


/**
 * @brief Read ADC pin, returning float using linear calibration with slope and offset
 * 
 * @param pin, The ADC pin to be read. 0-7
 * @param vref, The voltage reference. 0 = 1.1, 1 = external, >=2 = Vcc
 * @param slope, The slope (m) of the linear calibration curve f(x) = m*x + c
 * @param offset, The offset/zero crossing (c) of the linear calibration curve f(x) = m*x + c
 * @return float, The linearly calibrated ADC value
 **/ 
float read_ADc_pin_linearFunc(uint8_t pin, uint8_t vref, float slope, float offset);
