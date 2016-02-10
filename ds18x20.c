/*******************************************************************************
Title:    DS18X20-Functions via One-Wire-Bus
Author:   Martin Thomas <eversmith@heizung-thomas.de>   
          http://www.siwawi.arubi.uni-kl.de/avr-projects
Software: avr-gcc 4.3.3 / avr-libc 1.6.7 (WinAVR 3/2010) 
Hardware: any AVR - tested with ATmega16/ATmega32/ATmega324P and 3 DS18B20

Partly based on code from Peter Dannegger and others.

changelog:
20041124 - Extended measurements for DS18(S)20 contributed by Carsten Foss (CFO)
200502xx - function DS18X20_read_meas_single
20050310 - DS18x20 EEPROM functions (can be disabled to save flash-memory)
           (DS18X20_EEPROMSUPPORT in ds18x20.h)
20100625 - removed inner returns, added static function for read scratchpad
         . replaced full-celcius and fractbit method with decicelsius
           and maxres (degreeCelsius*10e-4) functions, renamed eeprom-functions,
           delay in recall_e2 replaced by timeout-handling
20100714 - ow_command_skip_last_recovery used for parasite-powerd devices so the
           strong pull-up can be enabled in time even with longer OW recovery
           times
20110209 - fix in DS18X20_format_from_maxres() by Marian Kulesza
20160203 - Add extra functionality for headless application. Decouple from USART
*******************************************************************************/

#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "ds18x20.h"
#include "usart.h"

#if DS18X20_EEPROMSUPPORT
// for 10ms delay in copy scratchpad
#include <util/delay.h>
#endif /* DS18X20_EEPROMSUPPORT */

/* find DS18X20 Sensors on 1-Wire-Bus
   input/ouput: diff is the result of the last rom-search
                *diff = OW_SEARCH_FIRST for first call
   output: id is the rom-code of the sensor found */
uint8_t DS18X20_find_sensor( uint8_t *diff, uint8_t id[] )
{
	uint8_t go;
	uint8_t ret;

	ret = DS18X20_OK;
	go = 1;
	do {
		*diff = ow_rom_search( *diff, &id[0] );
		if ( *diff == OW_PRESENCE_ERR || *diff == OW_DATA_ERR ||
		     *diff == OW_LAST_DEVICE ) { 
			go  = 0;
			ret = DS18X20_ERROR;
		} else {
			if ( id[0] == DS18B20_FAMILY_CODE || id[0] == DS18S20_FAMILY_CODE ||
			     id[0] == DS1822_FAMILY_CODE ) { 
				go = 0;
			}
		}
	} while (go);

	return ret;
}

/* get power status of DS18x20 
   input:   id = rom_code 
   returns: DS18X20_POWER_EXTERN or DS18X20_POWER_PARASITE */
uint8_t DS18X20_get_power_status( uint8_t id[] )
{
	uint8_t pstat;

	ow_reset();
	ow_command( DS18X20_READ_POWER_SUPPLY, id );
	pstat = ow_bit_io( 1 );
	ow_reset();
	return ( pstat ) ? DS18X20_POWER_EXTERN : DS18X20_POWER_PARASITE;
}

/* start measurement (CONVERT_T) for all sensors if input id==NULL 
   or for single sensor where id is the rom-code */
uint8_t DS18X20_start_meas( uint8_t with_power_extern, uint8_t id[])
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		if ( with_power_extern != DS18X20_POWER_EXTERN ) {
			ow_command_with_parasite_enable( DS18X20_CONVERT_T, id );
			/* not longer needed: ow_parasite_enable(); */
		} else {
			ow_command( DS18X20_CONVERT_T, id );
		}
		ret = DS18X20_OK;
	} 
	else { 
        // Verbose command, replace
// 		uart_puts_P_verbose( "DS18X20_start_meas: Short Circuit!\r" );
		ret = DS18X20_START_FAIL;
	}

	return ret;
}

// returns 1 if conversion is in progress, 0 if finished
// not available when parasite powered.
uint8_t DS18X20_conversion_in_progress(void)
{
	return ow_bit_io( 1 ) ? DS18X20_CONVERSION_DONE : DS18X20_CONVERTING;
}

static uint8_t read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n )
{
	uint8_t i;
	uint8_t ret;

	ow_command( DS18X20_READ, id );
	for ( i = 0; i < n; i++ ) {
		sp[i] = ow_byte_rd();
	}
	// TODO: Commented out crc8 as busted
// 	if ( crc8( &sp[0], DS18X20_SP_SIZE ) ) {
//     if ( 0 ) {
// 		ret = DS18X20_ERROR_CRC;
// 	} else {
		ret = DS18X20_OK;
// 	}

	return ret;
}


#if DS18X20_DECICELSIUS

/* convert scratchpad data to physical value in unit decicelsius */
static int16_t DS18X20_raw_to_decicelsius( uint8_t familycode, uint8_t sp[] )
{
	uint16_t measure;
	uint8_t  negative;
	int16_t  decicelsius;
	uint16_t fract;

	measure = sp[0] | (sp[1] << 8);
	//measure = 0xFF5E; // test -10.125
	//measure = 0xFE6F; // test -25.0625

	if( familycode == DS18S20_FAMILY_CODE ) {   // 9 -> 12 bit if 18S20
		/* Extended measurements for DS18S20 contributed by Carsten Foss */
		measure &= (uint16_t)0xfffe;   // Discard LSB, needed for later extended precicion calc
		measure <<= 3;                 // Convert to 12-bit, now degrees are in 1/16 degrees units
		measure += (16 - sp[6]) - 4;   // Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative 
	if ( measure & 0x8000 )  {
		negative = 1;       // mark negative
		measure ^= 0xffff;  // convert to positive => (twos complement)++
		measure++;
	}
	else {
		negative = 0;
	}

	// clear undefined bits for DS18B20 != 12bit resolution
	if ( familycode == DS18B20_FAMILY_CODE || familycode == DS1822_FAMILY_CODE ) {
		switch( sp[DS18B20_CONF_REG] & DS18B20_RES_MASK ) {
		case DS18B20_9_BIT:
			measure &= ~(DS18B20_9_BIT_UNDF);
			break;
		case DS18B20_10_BIT:
			measure &= ~(DS18B20_10_BIT_UNDF);
			break;
		case DS18B20_11_BIT:
			measure &= ~(DS18B20_11_BIT_UNDF);
			break;
		default:
			// 12 bit - all bits valid
			break;
		}
	}

	decicelsius = (measure >> 4);
	decicelsius *= 10;

	// decicelsius += ((measure & 0x000F) * 640 + 512) / 1024;
	// 625/1000 = 640/1024
	fract = ( measure & 0x000F ) * 640;
	if ( !negative ) {
		fract += 512;
	}
	fract /= 1024;
	decicelsius += fract;

	if ( negative ) {
		decicelsius = -decicelsius;
	}

	if ( /* decicelsius == 850 || */ decicelsius < -550 || decicelsius > 1250 ) {
		return DS18X20_INVALID_DECICELSIUS;
	} else {
		return decicelsius;
	}
}

/* format decicelsius-value into string, itoa method inspired 
   by code from Chris Takahashi for the MSP430 libc, BSD-license 
   modifications mthomas: variable-types, fixed radix 10, use div(), 
   insert decimal-point */
uint8_t DS18X20_format_from_decicelsius( int16_t decicelsius, char str[], uint8_t n)
{
	uint8_t sign = 0;
	char temp[7];
	int8_t temp_loc = 0;
	uint8_t str_loc = 0;
	div_t dt;
	uint8_t ret;

	// range from -550:-55.0°C to 1250:+125.0°C -> min. 6+1 chars
	if ( n >= (6+1) && decicelsius > -1000 && decicelsius < 10000 ) {

		if ( decicelsius < 0) {
			sign = 1;
			decicelsius = -decicelsius;
		}

		// construct a backward string of the number.
		do {
			dt = div(decicelsius,10);
			temp[temp_loc++] = dt.rem + '0';
			decicelsius = dt.quot;
		} while ( decicelsius > 0 );

		if ( sign ) {
			temp[temp_loc] = '-';
		} else {
			///temp_loc--;
			temp[temp_loc] = '+';
		}

		// reverse the string.into the output
		while ( temp_loc >=0 ) {
			str[str_loc++] = temp[(uint8_t)temp_loc--];
			if ( temp_loc == 0 ) {
				str[str_loc++] = DS18X20_DECIMAL_CHAR;
			}
		}
		str[str_loc] = '\0';

		ret = DS18X20_OK;
	} else {
		ret = DS18X20_ERROR;
	}
	
	return ret;
}

/* reads temperature (scratchpad) of sensor with rom-code id
   output: decicelsius 
   returns DS18X20_OK on success */
uint8_t DS18X20_read_decicelsius( uint8_t id[], int16_t *decicelsius )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ow_reset();
	ret = read_scratchpad( id, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
		*decicelsius = DS18X20_raw_to_decicelsius( id[0], sp );
	}
	return ret;
}

/* reads temperature (scratchpad) of sensor without id (single sensor)
   output: decicelsius 
   returns DS18X20_OK on success */
uint8_t DS18X20_read_decicelsius_single( uint8_t familycode, int16_t *decicelsius )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ret = read_scratchpad( NULL, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
		*decicelsius = DS18X20_raw_to_decicelsius( familycode, sp );
	}
	return ret;
}

#endif /* DS18X20_DECICELSIUS */


#if DS18X20_MAX_RESOLUTION

static int32_t DS18X20_raw_to_maxres( uint8_t familycode, uint8_t sp[] )
{
	uint16_t measure;
	uint8_t  negative;
	int32_t  temperaturevalue;

	measure = sp[0] | (sp[1] << 8);
	//measure = 0xFF5E; // test -10.125
	//measure = 0xFE6F; // test -25.0625

	if( familycode == DS18S20_FAMILY_CODE ) {   // 9 -> 12 bit if 18S20
		/* Extended measurements for DS18S20 contributed by Carsten Foss */
		measure &= (uint16_t)0xfffe;   // Discard LSB, needed for later extended precicion calc
		measure <<= 3;                 // Convert to 12-bit, now degrees are in 1/16 degrees units
		measure += ( 16 - sp[6] ) - 4; // Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative 
	if ( measure & 0x8000 )  {
		negative = 1;       // mark negative
		measure ^= 0xffff;  // convert to positive => (twos complement)++
		measure++;
	}
	else {
		negative = 0;
	}

	// clear undefined bits for DS18B20 != 12bit resolution
	if ( familycode == DS18B20_FAMILY_CODE || familycode == DS1822_FAMILY_CODE ) {
		switch( sp[DS18B20_CONF_REG] & DS18B20_RES_MASK ) {
		case DS18B20_9_BIT:
			measure &= ~(DS18B20_9_BIT_UNDF);
			break;
		case DS18B20_10_BIT:
			measure &= ~(DS18B20_10_BIT_UNDF);
			break;
		case DS18B20_11_BIT:
			measure &= ~(DS18B20_11_BIT_UNDF);
			break;
		default:
			// 12 bit - all bits valid
			break;
		}
	}

	temperaturevalue  = (measure >> 4);
	temperaturevalue *= 10000;
	temperaturevalue +=( measure & 0x000F ) * DS18X20_FRACCONV;

	if ( negative ) {
		temperaturevalue = -temperaturevalue;
	}

	return temperaturevalue;
}

uint8_t DS18X20_read_maxres( uint8_t id[], int32_t *temperaturevalue )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ow_reset();
	ret = read_scratchpad( id, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
		*temperaturevalue = DS18X20_raw_to_maxres( id[0], sp );
	}
	return ret;
}

uint8_t DS18X20_read_maxres_single( uint8_t familycode, int32_t *temperaturevalue )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ret = read_scratchpad( NULL, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
		*temperaturevalue = DS18X20_raw_to_maxres( familycode, sp );
	}
	return ret;

}

uint8_t DS18X20_format_from_maxres( int32_t temperaturevalue, char str[], uint8_t n)
{
	uint8_t sign = 0;
	char temp[10];
	int8_t temp_loc = 0;
	uint8_t str_loc = 0;
	ldiv_t ldt;
	uint8_t ret;

	// range from -550000:-55.0000°C to 1250000:+125.0000°C -> min. 9+1 chars
	if ( n >= (9+1) && temperaturevalue > -1000000L && temperaturevalue < 10000000L ) {

		if ( temperaturevalue < 0) {
			sign = 1;
			temperaturevalue = -temperaturevalue;
		}

		do {
			ldt = ldiv( temperaturevalue, 10 );
			temp[temp_loc++] = ldt.rem + '0';
			temperaturevalue = ldt.quot;
		} while ( temperaturevalue > 0 );
		
		// mk 20110209
		if ((temp_loc < 4)&&(temp_loc > 1)) {
			temp[temp_loc++] = '0';
		} // mk end

		if ( sign ) {
			temp[temp_loc] = '-';
		} else {
			temp[temp_loc] = '+';
		}

		while ( temp_loc >= 0 ) {
			str[str_loc++] = temp[(uint8_t)temp_loc--];
			if ( temp_loc == 3 ) {
				str[str_loc++] = DS18X20_DECIMAL_CHAR;
			}
		}
		str[str_loc] = '\0';

		ret = DS18X20_OK;
	} else {
		ret = DS18X20_ERROR;
	}
	
	return ret;
}

#endif /* DS18X20_MAX_RESOLUTION */


#if DS18X20_EEPROMSUPPORT

uint8_t DS18X20_write_scratchpad( uint8_t id[], 
	uint8_t th, uint8_t tl, uint8_t conf)
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_WRITE_SCRATCHPAD, id );
		ow_byte_wr( th );
		ow_byte_wr( tl );
		if ( id[0] == DS18B20_FAMILY_CODE || id[0] == DS1822_FAMILY_CODE ) {
			ow_byte_wr( conf ); // config only available on DS18B20 and DS1822
		}
		ret = DS18X20_OK;
	} 
	else { 
        // TODO: Deal with this
// 		uart_puts_P_verbose( "DS18X20_write_scratchpad: Short Circuit!\r" );
		ret = DS18X20_ERROR;
	}

	return ret;
}

uint8_t DS18X20_read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n )
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ret = read_scratchpad( id, sp, n );
	} 
	else { 
        // TODO: Deal with this
//uart_puts_P_verbose( "DS18X20_read_scratchpad: Short Circuit!\r" );
		ret = DS18X20_ERROR;
	}

	return ret;
}

uint8_t DS18X20_scratchpad_to_eeprom( uint8_t with_power_extern, 
	uint8_t id[] )
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		if ( with_power_extern != DS18X20_POWER_EXTERN ) {
			ow_command_with_parasite_enable( DS18X20_COPY_SCRATCHPAD, id );
			/* not longer needed: ow_parasite_enable(); */
		} else {
			ow_command( DS18X20_COPY_SCRATCHPAD, id );
		}
		_delay_ms(DS18X20_COPYSP_DELAY); // wait for 10 ms 
		if ( with_power_extern != DS18X20_POWER_EXTERN ) {
			ow_parasite_disable();
		}
		ret = DS18X20_OK;
	} 
	else { 
        // TODO: Deal with this
        //uart_puts_P_verbose( "DS18X20_copy_scratchpad: Short Circuit!\r" );
		ret = DS18X20_START_FAIL;
	}

	return ret;
}

uint8_t DS18X20_eeprom_to_scratchpad( uint8_t id[] )
{
	uint8_t ret;
	uint8_t retry_count=255;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_RECALL_E2, id );
		while( retry_count-- && !( ow_bit_io( 1 ) ) ) { 
			;
		}
		if ( retry_count ) {
			ret = DS18X20_OK;
		} else { 
        // TODO: Deal with this
        //uart_puts_P_verbose( "DS18X20_recall_E2: timeout!\r" );
			ret = DS18X20_ERROR;
		}
	} 
	else { 
        // TODO: Deal with this
        //uart_puts_P_verbose( "DS18X20_recall_E2: Short Circuit!\r" );
		ret = DS18X20_ERROR;
	}

	return ret;
}

#endif /* DS18X20_EEPROMSUPPORT */

/* DS18X20 functions by JM for use in headless control application 
 * At this stage these functions assume One OW bus,
 * It needs to be expanded to support multiple busses on different pins
 * 
 * These are _higher level_ functions and can be broken into two groups
 * Synchronous functions for standalone operation (slow, 12bit takes 750ms per
 * device!!
 * Asynchronous functions for temperature conversion from ISR
 */

uint8_t tSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
uint8_t tSensorPower[MAXSENSORS];
uint8_t ds18x20nSensors = 0;
uint8_t ds18x20Resolution = 0;
uint8_t ds18x20Toggle = 0;

/** Initialisation functions /*

/**
    @brief Scan the OW bus for DS18X20 temperature sensors, store addresses in
    tSensorIDs and power mode in tSensorPower
    @return ds18x20nSensors, number of sensors found
 */
static uint8_t search_sensors(void)
{
    uint8_t i;
    uint8_t id[OW_ROMCODE_SIZE];
    uint8_t diff;
    
    ow_reset();

    ds18x20nSensors = 0;
    
    diff = OW_SEARCH_FIRST;
    while ( diff != OW_LAST_DEVICE && ds18x20nSensors < MAXSENSORS ) {
        DS18X20_find_sensor( &diff, &id[0] );
        
        if( diff == OW_PRESENCE_ERR ) {
//             send_string( "No Sensor found" NEWLINESTR );
            break;
        }
        
        if( diff == OW_DATA_ERR ) {
//             send_string( "Bus Error" NEWLINESTR );
            break;
        }
        
        for ( i=0; i < OW_ROMCODE_SIZE; i++ )
            tSensorIDs[ds18x20nSensors][i] = id[i];
        tSensorPower[ds18x20nSensors] = DS18X20_get_power_status( 
            &tSensorIDs[ds18x20nSensors][0] );
        ds18x20nSensors++;
    }
    
    ow_reset();
    return ds18x20nSensors;
}

/**
    @brief initialize the OW bus detecting DS18X20 devices storing addr. and num
    @param resolution, Resolution has a great impact on conversion speed.
    @return status
 */
uint8_t ds18x20_init(uint8_t resolution) {
    //TODO: how to make this work for more than one bus...
    // initialize the bus (settings)
    uint8_t ret, sp[DS18X20_SP_SIZE], spT[DS18X20_SP_SIZE], i;
    ret = 0;
    // TODO: remove this and replace with scratchpad resolution
    ds18x20Resolution = resolution;
    
    #ifndef OW_ONE_BUS
        // TODO: Make this dynamic!
        ow_set_bus(&OWIN,&OWOUT,&OWDDR,OWPIN);;
    #endif

    // scan the bus saving addresses
    if (search_sensors() >= 1) {
        //set the resolution.
        for ( i = 0; i < ds18x20nSensors; i++ ) {
            // save the Th & Tl registers
            DS18X20_read_scratchpad( &tSensorIDs[i][0], sp, DS18X20_SP_SIZE );
            // Check if we need to change the config register
            if ( (sp[DS18B20_CONF_REG] & DS18B20_RES_MASK) != resolution) {
                // Write out the new config to scratchpad and eeprom
                DS18X20_write_scratchpad( &tSensorIDs[i][0], sp[DS18X20_TH_REG],
                                        sp[DS18X20_TL_REG], resolution );
                DS18X20_eeprom_to_scratchpad(&tSensorIDs[1][0]);
                DS18X20_read_scratchpad( &tSensorIDs[i][0], spT,
                                        DS18X20_SP_SIZE );
                // Check if the chane was actually made
                if ((spT[DS18B20_CONF_REG] & DS18B20_RES_MASK) == 
                        (sp[DS18B20_CONF_REG] & DS18B20_RES_MASK) ) {
                    ret = 0;
                }
            }
        }
    } else {
        ret = 1;
    }
    return ret;
}

/** Synchronous functions */

/**
    @brief Find the maximum temperature of all attached DS18X20 devices 
    @param  temperaturevalue, Maximum temperature of all DS18X20 devices
    @return Status
 */
uint8_t ds18x20_maxt_full(int32_t *temperaturevalue){
    int32_t temp_eminus4[ds18x20nSensors], temp_temp;
    uint8_t i, errors, timeConv;    
    
    errors = 0;
            
    // Read temperature from all sensors if resolution is 12 bit
    // Start conversion on all sensors
    for ( i = 0; i < ds18x20nSensors; i++ ) {
        DS18X20_start_meas( tSensorPower[i], &tSensorIDs[i][0] );
    }
    // delay for the conversion time
    switch (ds18x20Resolution) {
        case DS18B20_12_BIT:
           _delay_ms(DS18B20_TCONV_12BIT);
            break;
        case DS18B20_11_BIT:
           _delay_ms(DS18B20_11_BIT);
            break;
        case DS18B20_10_BIT:
           _delay_ms(DS18B20_10_BIT);
            break;
        case DS18B20_9_BIT:
           _delay_ms(DS18B20_9_BIT);
            break;
        default:
           _delay_ms(DS18B20_TCONV_12BIT);
    }
    // Read result on all sensors
    for ( i = 0; i < ds18x20nSensors; i++ ) {
        if ( DS18X20_read_maxres( &tSensorIDs[i][0], &temp_temp )
            == DS18X20_OK ) {
            temp_eminus4[i] = temp_temp;
        } else {
            temp_eminus4[i] = 0;
            errors++;
        }
    }
    
    // Error if no temperature measurements are good return 0 value
    if (errors<ds18x20nSensors) {
        // Find the maximum value
        temp_temp = 0;
        for (i = 0; i < ds18x20nSensors; i++) {
            if ( temp_eminus4[i] > temp_temp ) {
                temp_temp = temp_eminus4[i];
            }
        }
        *temperaturevalue = temp_temp;
    } else {
        *temperaturevalue = 0;
    }
    return errors;
}

/**
    @brief Find the maximum temperature of all attached DS18X20 devices in deciC
    @param  devices, Array of DS18X20 device addresses
    @return Maximum temperature of all DS18X20 devices in deci Centigrade
 */
int16_t ds18x20_maxt_deciC(int16_t *temperaturevalue){
    int16_t temp_deciC[ds18x20nSensors], temp_temp;
    uint8_t i, errors, timeConv;    
    
    errors = 0;
               
    // Read temperature from all sensors if resolution is 12 bit
    // Start conversion on all sensors
    for ( i = 0; i < ds18x20nSensors; i++ ) {
        DS18X20_start_meas( tSensorPower[i], &tSensorIDs[i][0] );
    }
    // delay for the conversion time
    switch (ds18x20Resolution) {
        case DS18B20_12_BIT:
           _delay_ms(DS18B20_TCONV_12BIT);
            break;
        case DS18B20_11_BIT:
           _delay_ms(DS18B20_11_BIT);
            break;
        case DS18B20_10_BIT:
           _delay_ms(DS18B20_10_BIT);
            break;
        case DS18B20_9_BIT:
           _delay_ms(DS18B20_9_BIT);
            break;
        default:
           _delay_ms(DS18B20_TCONV_12BIT);
    }
    // Read result on all sensors
    for ( i = 0; i < ds18x20nSensors; i++ ) {
        if ( DS18X20_read_decicelsius( &tSensorIDs[i][0], &temp_temp )
            == DS18X20_OK ) {
            temp_deciC[i] = temp_temp;
        } else {
            temp_deciC[i] = 0;
            errors++;
        }
    }
    
    // Error if no temperature measurements are good return 0 value
    if (errors<ds18x20nSensors) {
        // Find the maximum value
        temp_temp = 0;
        for (i = 0; i < ds18x20nSensors; i++) {
            if ( temp_deciC[i] > temp_temp ) {
                temp_temp = temp_deciC[i];
            }
        }
        *temperaturevalue = temp_temp;
    } else {
        *temperaturevalue = 0;
    }
    return errors;
}

/** Asynchronous functions */
//Toggle functions, take two runs to get result assuming called greater than
// the conversion time between.
/**
    @brief Find the maximum temperature of all attached DS18X20 devices where
    the first run initialises conversion and the second gets results
    @param  temperaturevalue, Maximum temperature of all DS18X20 devices
    @return Status
 */
uint8_t ds18x20_maxt_full_toggle(int32_t *temperaturevalue){
    int32_t temp_eminus4[ds18x20nSensors], temp_temp;
    uint8_t i, errors;    
    
    errors = 0;
    
    if (ds18x20Toggle) {
        
        // Start conversion on all sensors
        for ( i = 0; i < ds18x20nSensors; i++ ) {
            DS18X20_start_meas( tSensorPower[i], &tSensorIDs[i][0] );
        }
        ds18x20Toggle = !ds18x20Toggle;
    } else {
        // Read result on all sensors
        for ( i = 0; i < ds18x20nSensors; i++ ) {
            if ( DS18X20_read_maxres( &tSensorIDs[i][0], &temp_temp )
                == DS18X20_OK ) {
                temp_eminus4[i] = temp_temp;
            } else {
                temp_eminus4[i] = 0;
                errors++;
            }
        }
        // Error if no temperature measurements are good return 0 value
        if (errors<ds18x20nSensors) {
            // Find the maximum value
            temp_temp = 0;
            for (i = 0; i < ds18x20nSensors; i++) {
                if ( temp_eminus4[i] > temp_temp ) {
                    temp_temp = temp_eminus4[i];
                }
            }
            *temperaturevalue = temp_temp;
        } else {
            *temperaturevalue = 0;
        }
        ds18x20Toggle = !ds18x20Toggle;
    }
    return errors;
}

/**
    @brief Find the maximum temperature of all attached DS18X20 devices where
    the first run initialises conversion and the second gets results in deciC
    @param  temperaturevalue, Maximum temperature of all DS18X20 devices
    @return Maximum temperature of all DS18X20 devices in deci Centigrade
 */
int16_t ds18x20_maxt_deciC_toggle(int16_t *temperaturevalue){
    int16_t temp_deciC[ds18x20nSensors], temp_temp;
    uint8_t i, errors;    
    
    errors = 0;
    
    if (ds18x20Toggle) {\
        // Start conversion on all sensors
        for ( i = 0; i < ds18x20nSensors; i++ ) {
            DS18X20_start_meas( tSensorPower[i], &tSensorIDs[i][0] );
        }
        ds18x20Toggle = !ds18x20Toggle;
    } else {
        // Read result on all sensors
        for ( i = 0; i < ds18x20nSensors; i++ ) {
            if ( DS18X20_read_decicelsius( &tSensorIDs[i][0], &temp_temp )
                == DS18X20_OK ) {
                temp_deciC[i] = temp_temp;
            } else {
                temp_deciC[i] = 0;
                errors++;
            }
        }
        
        // Error if no temperature measurements are good return 0 value
        if (errors<ds18x20nSensors) {
            // Find the maximum value
            temp_temp = 0;
            for (i = 0; i < ds18x20nSensors; i++) {
                if ( temp_deciC[i] > temp_temp ) {
                    temp_temp = temp_deciC[i];
                }
            }
            *temperaturevalue = temp_temp;
        } else {
            *temperaturevalue = 0;
        }
        ds18x20Toggle = !ds18x20Toggle;
    }
    return errors;
}