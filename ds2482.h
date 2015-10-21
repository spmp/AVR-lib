/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file ds2482.h
 *
 * @brief Header for ds2482.c, driver/high level commands for the ds2482 I2C to 1wire bus device
 *
 * - Compiler:          gcc-avr
 * - Project:           AVR-lib
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: October 2015
 *****************************************************************************/ 
#pragma once
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "i2c_safe.h"

#define DS2482_CONFIG_APU (1<<0)  // Active pull up
#define DS2482_CONFIG_PPM (1<<1)  // No pull up 
#define DS2482_CONFIG_SPU (1<<2)  // Strong pull up
#define DS2484_CONFIG_WS  (1<<3)  // 1-Wire speed

#define DS2482_STATUS_BUSY  (1<<0)
#define DS2482_STATUS_PPD   (1<<1)
#define DS2482_STATUS_SD    (1<<2)
#define DS2482_STATUS_LL    (1<<3)
#define DS2482_STATUS_RST   (1<<4)
#define DS2482_STATUS_SBR   (1<<5)
#define DS2482_STATUS_TSB   (1<<6)
#define DS2482_STATUS_DIR   (1<<7)

extern uint8_t searchAddress[8];
extern uint8_t searchLastDisrepancy;
extern uint8_t searchExhausted;
extern uint8_t ds2482Address;
    
/**
    @brief      Reset the DS2482 device at the given I2C address
    @param      address 7 bit I2C address (in decimal)
    @return     status  1 success, 0 failure
*/
uint8_t ds2482_reset(uint8_t address);

/** 
 * ds2482_configure
 * @brief configure the DS2483 mode
 * 
 * @param addres the I2C address of the DS2482 device
 * @param config Must be one of DS2482_CONFIG_*
 * @return 1 success, 0 failure of some kind (usually no device)
 **/
uint8_t ds2482_configure(uint8_t address, uint8_t config);

/** 
 * ds2482_wireReset
 * @brief reset the one wire (OW) bus
 * 
 * @param addres the I2C address of the DS2482 device
 * @return 1 success, 0 failure of some kind (usually no device)
 **/
uint8_t ds2482_wireReset(uint8_t address);
uint8_t ds2482_wireReset_verbose(uint8_t address);

/** 
 * ds2482_wireWriteByte
 * @brief Write a byte to the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @param byte The byte to write to the OW bus
 * @return 1 success, 0 failure
 **/
uint8_t ds2482_wireWriteByte(uint8_t address, uint8_t byte);

/** 
 * ds2482_wireReadByte
 * @brief Read a byte from the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @return byte from OW bus or I2C_SAFE_READ_ERROR_CODE if error
 **/
uint8_t ds2482_wireReadByte(uint8_t address);

/** 
 * ds2482_wireReadBit
 * @brief Read a bit from the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @return bit from OW bus
 **/
uint8_t ds2482_wireReadBit(uint8_t address);

/** 
 * ds2482_wireWriteBit
 * @brief Write a single bit to the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @param bit The bit to write to the OW bus
 * @return 1 success, 0 failure
 **/
uint8_t ds2482_wireWriteBit(uint8_t address, uint8_t bit);

void ds2482_wireSkip(uint8_t address);

/** 
 * ds2482_wireSelect
 * @brief Select the OW device
 * 
 * @param addres The I2C address of the DS2482 device
 * @param rom[8] The rom address of the OW device
 **/
void ds2482_wireSelect(uint8_t address, uint8_t *rom);

/** 
 * ds2482_wireSearch
 * @brief Scan the OW bus for devices, returning when a device is found
 *  Scans the OW bus from the DS2482 at 'address', stopping whenever a device
 *   is found, storing the result in *newAddr. When the bus is fully scanned
 *   'searchExhausted' is set and 1 is returned
 * 
 * @param addres The I2C address of the DS2482 device
 * @param *newAddr An array (uint8_t[8]) to store the address in
 * @return 0 if address is found, 1 if no devices on bus or search exhausted
 **/
uint8_t ds2482_wireSearch(uint8_t address, uint8_t *newAddr);
uint8_t ds2482_wireSearch_verbose(uint8_t address, uint8_t *newAddr);

uint8_t ds2482_read_word(uint8_t address, uint8_t reg);



// Very low level, use with caution!
uint8_t ds2482_readByte(uint8_t address);
uint8_t ds2482_wireReadStatus(uint8_t address, bool setPtr);
