/**
 * @file timers.h
 * @author Jasper Aorangi
 * @date 10 July 2014
 * @brief Header for timer related function for AVM
 *
 * This library encapsulates timer related operations for Atmel AVR.
 * Specifically for my projects using ATMEGA 328p.
 *  At this time, I have functions for a 'clock', a once per second
 *  'tick' and for PWM.
 * 
 */

#pragma once
#include <avr/io.h>

/**
 * @section clock
 * @brief Routines, defines and intialisation for 'clock' functions.
 * 
 * The 'clock' is a timeout to call an ISR which calls functions within
 *  the ISR depending on the timeperioud of interest.
 * The most basic and useful is a once per second callback
 * 
 * Resources used:
 *  Timer/Counter 2, "8-bit Timer/Counter2 with PWM and Asynchronous Operation"
 * 
 */

/* Medium and fast time intervals in 1/125'th of a second */
#define MEDIUM_TIME_INTERVAL    25
#define FAST_TIME_INTERVAL      5

/**
 * @var timestamp
 * @brief unix timestamp: seconds since epoch 
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
 * Resources used:
 *  16-bit Timer/Counter1 with PWM
 **/

/**
 * @brief Initialise the PWM
 * @return none
 * 
 * @todo Include parameters for frequency etc etc etc in PWM. Probably from #defines
 **/
void init_pwm();

/**
 * @brief Get the value of the PWM duty cycle register
 * @return unit16 the value of the PWM duty cycle
 **/
uint16_t get_pwm();

/**
 * @brief Set the value of the PWM duty cycle register
 * @param duty, The value of the PWM duty cycle. uint16
 **/
void set_pwm(uint16_t duty);
