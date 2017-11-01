/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file adc.c
 *
 * @brief Functions for accessig the internal ADC's
 *
 * - Compiler:          gcc-avr
 * - Project:           AVR-lib
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: 2017
 *****************************************************************************/
#include "adc.h"

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

/**
 * @brief Read ADC pin, returning value
 * 
 * @param pin, The ADC pin to be read. 0-7
 * @param vref, The voltage reference. 0 = 1.1, 1 = external, >=2 = Vcc
 * @return uint16, The 10bit value from the init_ADC
 **/ 
uint16_t read_ADC_pin(uint8_t pin, uint8_t vref)
{
    //Clear the Power reduction register to enable the ADC in case we have gone to sleep between ADC reads
    PRR &= ~(1 << PRADC);
    
    //According to Vref, start the ADC read on pin
    switch(vref) {
        // 1.1V Vref
        case 0:
            ADMUX = (1 << REFS0) | (1 << REFS1) | (pin & 0xF);
            break;
        // External Vref
        case 1:
            ADMUX = (pin & 0xF);
            break;
        // Defaults to VCC as Vref
        default :
            ADMUX = (1 << REFS0)| (pin & 0xF);
            break;
    }
    
    // Enable the ADC, start measuring whilst setting the prescaler to 128
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    //Wait for conversion to finish
    while ((ADCSRA & (1 << ADSC)) != 0);
    
    //Return the ADC code
    return ADC;
}

/**
 * @brief Read ADC pin, returning a value in millivolts
 * 
 * @param pin, The ADC pin to be read. 0-7
 * @param vref, The voltage reference. 0 = 1.1, 1 = external, >=2 = Vcc
 * @return uint16, ADC reading in millivolts
 **/
uint16_t read_ADC_pin_millivolts(uint8_t pin, uint8_t vref)
{
    //Clear the Power reduction register to enable the ADC in case we have gone to sleep between ADC reads
    PRR &= ~(1 << PRADC);
    
    //According to Vref, start the ADC read on pin
    switch(vref) {
        // 1.1V Vref
        case 0:
            ADMUX = (1 << REFS0) | (1 << REFS1) | (pin & 0xF);
            break;
        // External Vref
        case 1:
            ADMUX = (pin & 0xF);
            break;
        // Defaults to VCC as Vref
        default :
            ADMUX = (1 << REFS0) | (pin & 0xF);
            break;
    }
    
    // Enable the ADC, start measuring whilst setting the prescaler to 128
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    //Wait for conversion to finish
    while ((ADCSRA & (1 << ADSC)) != 0);
    
    //Return the ADC millivolts according to Vref, casting the calculation into 32bit to avoid overflow
    switch(vref) {
        // 1.1V Vref
        case 0:
            return ((uint32_t)ADC*1100)/1024;
            break;
        // External Vref
        case 1:
            return ((uint32_t)ADC*EXTERNAL_MILLIVOLT)/1024;
            break;
        // Defaults to VCC as Vref
        default :
            return ((uint32_t)ADC*5000)/1024;
            break;
    }
}

/**
 * @brief Read ADC pin, returning float using linear calibration with slope and offset
 * 
 * @param pin, The ADC pin to be read. 0-7
 * @param vref, The voltage reference. 0 = 1.1, 1 = external, >=2 = Vcc
 * @param slope, The slope (m) of the linear calibration curve f(x) = m*x + c
 * @param offset, The offset/zero crossing (c) of the linear calibration curve f(x) = m*x + c
 * @return float, The linearly calibrated ADC value
 **/ 
float read_ADc_pin_linearFunc(uint8_t pin, uint8_t vref, float slope, float offset){
  return float(read_ADC_pin(pin, vref))*slope + offset;
}
