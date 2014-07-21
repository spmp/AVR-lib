#include <avr/interrupt.h>
#include "timers.h"

void (* oncePerSecondCallback)();       // void function pointer
void (* fastCallback)();                // void function pointer
void (* mediumCallback)();                // void function pointer

uint32_t timestamp;
uint16_t ticks;
uint16_t mediumTime;
uint16_t fastTime;

void clock_set_seconds_callback(void (* secondsCallback)()) {
    oncePerSecondCallback = secondsCallback;
}

void clock_set_fast_time_callback(void (* fast_Callback)()) {
    fastCallback = fast_Callback;
}

void clock_set_medium_time_callback(void (* medium_Callback)()) {
    mediumCallback = medium_Callback;
}

ISR(TIMER2_COMPA_vect) {
    /* increment clock */
    ticks += 1;
    mediumTime +=1;
    fastCallback += 1; //Would be better to lump all this together and force medim and fast as a mod(125,fast/med time) check 
    
    if ( fastCallback && (fastTime >= FAST_TIME_INTERVAL) ) {
        fastTime = 0;
        fastCallback();
    }
    
    if (mediumCallback && (mediumTime >= MEDIUM_TIME_INTERVAL) ) {
        mediumTime = 0;
        mediumCallback();
    }
    
    if (ticks >= 125) {
        ticks = 0;
        timestamp += 1;
        if (timestamp >= 86401) { //roll over for 24 hours
            timestamp = 0;
        }
        if (oncePerSecondCallback) {
            oncePerSecondCallback();
        }
    }
}

void init_clock() {
    ticks = 0;
    timestamp = 0;

    TCCR2A = (1 << WGM21); /* clear timer on compare match */
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); /* 1024 pre-scaler */
    TIMSK2 = (1 << OCIE2A); /* compare match interrupt */
    OCR2A = 125 - 1;
}

/**************** PWM ********************************************************/
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
void init_pwm(uint16_t top, uint8_t channels)
{
    //Disable Power Reduction Register in order to enable Timer/Counter1
    // by clearing PRTIM1
    PPR &= !(1 << PRTIM1);
    
    //Set PB1 and PB2 (OC1A and OC1B) as outputs
    DDRB |= (1 << DDB1);
    if (channels >= 2){
        DDRB |= (1 << DDB2);
    }
    
    //Set TOP to top (0xFFFF is 16bit)
    OCR1 = top;
    
    //Set Duty Cylce to 0
    OCR1A = 0;
    OCR1B = 0;
    
    //Set non inverting mode on the output pin(s)
    TCCR1A = (1 << COM1A1);
    if (channels >= 2){
        TCCR1A |= (1 << COM1B1);
    }
    
    //Set PWM as Phase and Frequency correct mode with ICR1, mode 8
    TCCR1B = (1 << WGM13);
    
    //Set the clocksource to internal, and no prescalar. Starts the PWM.
    TCCR1B |= (1 << CS10);
}

/**
 * @brief Get the value of the PWM duty cycle register
 * @param channel, the PWM channel to get, 1 for A, 2 for B
 * @return unit16 the value of the PWM duty cycle
 **/
uint16_t get_pwm(uint8_t channel)
{
    if (channel < 2){
        return OCR1A;
    }
    else
        return OCR1B;
}

/**
 * @brief Set the value of the PWM duty cycle register
 * @param duty, The value of the PWM duty cycle. uint16
 **/
void set_pwm(uint8_t channel, uint16_t duty)
{
    if (channel < 2){
        OCR1A = duty;
    }
    else
        OCR1B = duty;
}
