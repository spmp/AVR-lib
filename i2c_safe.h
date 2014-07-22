#pragma once
/*******************************************************************************
 * High(er) Level I2C commands
 * 
 *  Higher level I2C commands and Safe I2C reads
 * 
 * Author: Jasper Aorangi
 * 
 * i2c_highlevel.h
 ******************************************************************************/

#include <avr/io.h>

/* Safe I2C reads
 *  i2c_read_... waits for the slave device to respond in a while loop.
 *   Should the slave address be wrong (easily detected when setting adress 
 *   pointer) or the communications be disrupted by noise or any other means,
 *   then the standard I2C read will hang indefinatly.
 *  i2c_safe_read... uses a timeout in the while loop to break out, returning
 *  I2C_SAFE_READ_ERROR_CODE instead of I2C read.
 *   This is not ideal as I2C_SAFE_READ_ERROR_CODE could be valid data, rather 
 *   if a struct with error code and data are returned, this would be more 
 *   reliable
 *  TODO: Return a struct for all I2C commands Possible call these files 
 *  I2C_safe_master
 ******************************************************************************/

/** defines the data direction (reading from I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_READ    1

/** defines the data direction (writing to I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_WRITE   0

#ifndef I2C_SAFE_TIMEOUT
#define I2C_SAFE_TIMEOUT 100
#endif

#ifndef I2C_SAFE_READ_ERROR_CODE
#define I2C_SAFE_READ_ERROR_CODE 0x00
#endif



/**
    @brief initialize the I2C master interace. Need to be called only once 
    @param  I2Cfreq, the frequqncy of the I2C port in Hz. Must be less than 444kHz
    @param  Fcpu, micro clock speed in Hz
    @return none
*/
extern void i2c_init(uint32_t I2CFreq, uint16_t Fcpu);


/** 
    @brief Terminates the data transfer and releases the I2C bus 
    @param void
    @return none
*/
extern void i2c_stop(void);


/** 
    @brief Issues a start condition and sends address and transfer direction 
    
    @param    addr address and transfer direction of I2C device
    @retval   0   device accessible 
    @retval   1   failed to access device 
*/
extern unsigned char i2c_start(unsigned char addr);


/**
    @brief Issues a repeated start condition and sends address and transfer direction 
    
    @param   addr address and transfer direction of I2C device
    @retval  0 device accessible
    @retval  1 failed to access device
*/
extern unsigned char i2c_rep_start(unsigned char addr);


/**
    @brief Issues a start condition and sends address and transfer direction 
     
    If device is busy, use ack polling to wait until device ready 
    @param    addr address and transfer direction of I2C device
    @return   none
*/
extern void i2c_start_wait(unsigned char addr);


/**
    @brief Send one byte to I2C device
    @param    data  byte to be transfered
    @retval   0 write successful
    @retval   1 write failed
*/
extern unsigned char i2c_write(unsigned char data);


/** @brief      Safe I2C read Ack
    @details    Read one byte from the I2C d*evice, request more data from
                device and return error if communication times out.
    @retval     byte read from I2C device if no timeout,
    @retval     I2C_SAFE_READ_ERROR_CODE if I2C_SAFE_TIMEOUT reached.
*/
 extern unsigned char i2c_safe_readAck(void);

 
/** @brief      Safe I2C read Nak
    @details    Read one byte from the I2C d*evic*e, read is followed by a stop
                condition and return error if communication times out.
    @retval     byte read from I2C device if no timeout,
    @retval     I2C_SAFE_READ_ERROR_CODE if I2C_SAFE_TIMEOUT reached.
*/
 extern unsigned char i2c_safe_readNak(void);

/** @brief      Read 8 bit word from I2C device, given address and register
    @param      address 7 bit I2C address
    @param      reg     Register address. Some devices are dumb and have no
    @retval     registervalue 8 bit word
    @retval     I2C_SAFE_READ_ERROR_CODE if read fails
*/
 uint8_t i2c_safe_read_word(unsigned char address, uint8_t reg);
 
/** @brief      Read 16 bit word from I2C device, given address and register
    @param      address 7 bit I2C address
    @param      reg     Register address. 0xFF is for no register for dumb device
    @retval     registervalue 16 bit word
    @retval     I2C_SAFE_READ_ERROR_CODE if read fails
*/
 uint16_t i2c_safe_read_sixteen(uint8_t address, uint8_t reg);
 
/** @brief      Write an 8 bit word to I2C device, given address and register
    @param      address 7 bit I2C address
    @param      reg     Register address. 0xFF is for no register for dumb device
    @param      data    8 bit uint
    @retval     error   3 error <start write><write adress><write data word>
*/
 uint8_t i2c_safe_write_word(uint8_t address, uint8_t reg, uint8_t data);
 
/** @brief      Write 16 bit word to I2C device, given address and register 
    @param      address 7 bit I2C address                                         *
    @param      reg     Register address
    @param      data    16 bit uint
    @retval     error   4 bit error code as: 
       <start write><write adress><write upper data word><write lower data word>
*/
 uint8_t i2c_safe_write_sixteen(uint8_t address, uint8_t reg, uint16_t data);
 
/** @brief      Scan the I2C bus for acknowledging devices in read mode
    @param      start_addr 7 bit I2C address to start scanning at start address
    @param      stop_addr  7 bit I2C address at which to stop scanning decriments
    @retval     address    7 bit I2C address of acknowledging device(s) via USART
    TODO: Place these in an array (of unknown size) to remove dependency on usart.h
*/
 void i2c_safe_read_scan_bus(uint8_t start_addr, uint8_t stop_addr);
 
/** @brief      Scan the I2C bus for acknowledging devices in write mode
    @param      start_addr 7 bit I2C address to start scanning at (highest value)
    @param      stop_addr  7 bit I2C address at which to stop scanning
    @retval     address    7 bit I2C address of acknowledging device(s) via USART
    TODO: Place these in an array (of unknown size) to remove dependency on usart.h
*/
 void i2c_safe_write_scan_bus(uint8_t start_addr, uint8_t stop_addr);
 