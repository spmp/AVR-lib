#include <avr/interrupt.h>
#include "clock.h"

void (* oncePerSecondCallback)();       // void function pointer
void (* fastCallback)();                // void function pointer
void (* mediumCallback)();                // void function pointer
void (* longCallback)();                // void function pointer

uint32_t timestamp;
uint16_t ticks;
uint16_t mediumTime;
uint32_t longTime;
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

void clock_set_long_time_callback(void (* long_Callback)()) {
    longCallback = long_Callback;
}

ISR(TIMER2_COMPA_vect) {
    /* increment clock */
    ticks += 1;
    longTime += 1;
    mediumTime +=1;
    fastCallback += 1;
    
    if ( fastCallback && (fastTime >= FAST_TIME_INTERVAL) ) {
        fastTime = 0;
        fastCallback();
    }
    
    if (mediumCallback && (mediumTime >= MEDIUM_TIME_INTERVAL) ) {
        mediumTime = 0;
        mediumCallback();
    }
    
    if ( longCallback && (longTime >= FAST_TIME_INTERVAL) ) {
        longTime = 0;
        longCallback();
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

    DDRB = (1 << DDB5);

    TCCR2A = (1 << WGM21); /* clear timer on compare match */
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); /* 1024 pre-scaler */
    TIMSK2 = (1 << OCIE2A); /* compare match interrupt */
    OCR2A = 125 - 1;
}

