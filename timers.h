/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file timers.h
 *
 * @brief Header for timers.c, Timers and related functions such as PWM
 *
 * - Compiler:          gcc-avr
 * - Project:           AVR-lib
 * - uC                 AVR
 * 
 * This library encapsulates timer related operations for Atmel AVR.
 * Specifically for my projects using ATMEGA 328p.
 *  At this time, I have functions for a 'clock', a once per second
 *  'tick' and for PWM.
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>

/**
 * @section clock
 * @brief Routines, defines and intialisation for 'clock' functions.
 * 
 * The 'clock' is a timeout to call an ISR which calls functions within
 *  the ISR depending on the timeperioud of interest.
 * The most basic and useful is a once per second callback
 * 
 * @resources Timer/Counter2, "8-bit Timer/Counter2 with PWM and Asynchronous Operation"
 * 
 */

/* Medium and fast time intervals in 1/125'th of a second */
#define MEDIUM_TIME_INTERVAL    25
#define FAST_TIME_INTERVAL      5

/**
 * @var timestamp
 * @brief unix timestamp: seconds since epoch, or at least startup 
 **/
extern uint32_t timestamp;

/**
 * @var ticks
 * @brief tick counter. 125 ticks per second 
 **/
extern uint16_t ticks;

/**
 * @brief initialize the clock
 * @return none
 **/
void init_clock();

/**
 * @brief Set the name of the function that gets called once every second. N.B. is called within an interrupt
 * @param  secondsCallback, the address of the function to run every second. i.e. &blahfunction
 * @return none
 **/
void clock_set_seconds_callback(void (* secondsCallback)());

/**
 * @brief Set the name of the function that gets called on the fast interval. N.B. is called within an interrupt
 * @param  fast_Callback, the address of the function to run on the fast interval. i.e. &blahfunction
 * @return none
 **/
void clock_set_fast_time_callback(void (* fast_Callback)());

/* callback that is called once every MEDIUM_TIME_INTERVAL. For closer to realtime activity
 * N.B. is called within an interrupt */
/**
 * @brief Set the name of the function that gets called in the medium timeframe. N.B. is called within an interrupt
 * @param  secondsCallback, the address of the function to run on the medium time interval. i.e. &blahfunction
 * @return none
 **/
void clock_set_medium_time_callback(void (* medium_Callback)());


/** 
 * @section PWM
 * @brief Routines, defines and initialisation for PWM functions
 * 
 * @resource Timer/Counter1, 16-bit Timer/Counter1 with PWM
 * @resource PB1/PB2
 **/

/**
 * @brief Initialise the PWM
 * @param top, The value of TOP for the PWM counter
 * @param channels, Number of PWM channels. Valid values are 1 or 2. 1 for PB1 and 2 for PB1 and PB2
 * @return none
 * 
 * @todo Include parameters for frequency etc etc etc in PWM. Probably from #defines
 **/
void init_pwm(uint16_t top, uint8_t channels);

/**
 * @brief Get the value of the PWM duty cycle register
 * @param channel, the PWM channel to get, 1 for A, 2 for B
 * @return, unit16 the value of the PWM duty cycle
 **/
uint16_t get_pwm(uint8_t channel);

/**
 * @brief Set the value of the PWM duty cycle register
 * @param channel, uint8 the PWM channel to get, 1 for A, 2 for B
 * @param duty, unint16, the value of the PWM duty cycle. uint16
 **/
void set_pwm(uint8_t channel, uint16_t duty);
