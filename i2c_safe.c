/*******************************************************************************
 * High(er) Level I2C commands
 * 
 *  Higher level I2C commands and Safe I2C reads
 * 
 * Low level commands based heavily on twimaster by Peter Fleury.
 * 
 * Author: Jasper Aorangi
 * 
 * i2c_highlevel.h
 ******************************************************************************/

#include <inttypes.h>
#include <compat/twi.h>

#include "i2c_safe.h"

#include "usart.h" //TODO: Remove this dependency!


/*************************************************************************
 * Initialization of the I2C bus interface. Need to be called only once
 *************************************************************************/
void i2c_init(uint32_t I2Cfreq, uint16_t fcpu)
{
    /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
    
    TWSR = (0<<TWPS1) | (0<<TWPS0);                  /* no prescaler */
    TWBR = ((fcpu/I2Cfreq)-16)/2;  /* must be > 10 for stable operation */
    
}/* i2c_init */


/*************************************************************************  
 I ssues a star*t condition and sends address and transfer direction.
 return 0 = device accessible, 1= failed to access device
 *************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;
    
    // send START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // wait until transmission completed
    
    while(!(TWCR & (1<<TWINT)));
    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;
    
    // send device address
    TWDR = address;
    TWCR = (1<<TWINT) | (1<<TWEN);
    
    // wail until transmission completed and ACK/NACK has been received
    while(!(TWCR & (1<<TWINT)));
    
    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
    
    return 0;
    
}/* i2c_start */


/*************************************************************************
 I ssues a start* condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
 *************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;
    
    while ( 1 )
    {
        // send START condition
        TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
        
        // wait until transmission completed
        while(!(TWCR & (1<<TWINT)));
        
        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
        
        // send device address
        TWDR = address;
        TWCR = (1<<TWINT) | (1<<TWEN);
        
        // wail until transmission completed
        while(!(TWCR & (1<<TWINT)));
        
        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
        {           
            /* device busy, send stop condition to terminate write operation */
            TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
            
            // wait until stop condition is executed and bus released
            while(TWCR & (1<<TWSTO));
            
            continue;
        }
        //if( twst != TW_MT_SLA_ACK) return 1;
        break;
    }
    
}/* i2c_start_wait */


/*************************************************************************
 I ssues a repeated start condition and sends address and transfer direction
 
 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
 1 failed to access device
 *************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );
    
}/* i2c_rep_start */

/*************************************************************************
 T erminates the* data transfer and releases the I2C bus
 *************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    
    // wait until stop condition is executed and bus released
    while(TWCR & (1<<TWSTO));
    
}/* i2c_stop */


/*************************************************************************
 S end one byte *to I2C device
 
 Input:    byte to be transfered
 Return:   0 write successful 
 1 write failed
 *************************************************************************/
unsigned char i2c_write( unsigned char data )
{   
    uint8_t   twst;
    
    // send data to the previously addressed device
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    
    // wait until transmission completed
    while(!(TWCR & (1<<TWINT)));
    
    // check value of TWI Status Register. Mask prescaler bits
    twst = TW_STATUS & 0xF8;
    if( twst != TW_MT_DATA_ACK) return 1;
    return 0;
    
}/* i2c_write */


/* Safe commands */

/** Safe I2C read Ack
 @brief         Read one byte from the I2C d*evic*e, request more data from device
                and return error if communication times out.
 @retval        byte read from I2C device if no timeout
 @retval        I2C_SAFE_READ_ERROR_CODE if I2C_SAFE_TIMEOUT reached
 TODO:          Make this safe with counter in while loop. BRENDAN HELP! it hangs if have a break or return from while/for/do while!
*/
unsigned char i2c_safe_readAck(void)
 {
     TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
     while(!(TWCR & (1<<TWINT)));
     
     return TWDR;
     
 }/* i2c_safe_readAck */

/**
    @brief      Safe I2C read Nak
    @details    Read one byte from the I2C device, read is followed by a stop condition
                and return error if communication times out.
    @retval     byte read from I2C device if no timeout,
    @retval     I2C_SAFE_READ_ERROR_CODE if I2C_SAFE_TIMEOUT reached
*/
unsigned char i2c_safe_readNak(void)
{
    uint8_t i2c_safe_timeout_counter = I2C_SAFE_TIMEOUT; 
    TWCR = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR & (1<<TWINT)) );
    
    return TWDR;
    
}/* i2c_safe_readNak */


/* Higher level commands */


/**
    @brief    Read 8 bit word from I2C device, given address and register
    @param      address 7 bit I2C address
    @param      reg     Register address. 0xFF is for no register for dumb device
    @retval     registervalue 8 bit word
    @retval     I2C_SAFE_READ_ERROR_CODE if read fails
*/
uint8_t i2c_safe_read_word(unsigned char address, uint8_t reg){
    uint8_t LowerByte;
    if(!i2c_start((address << 1)| I2C_WRITE)){ //Address of the device in write mode, and if device acknowledges, then continue
        if (reg != 0xFF) {
            i2c_write(reg); //Set the pointer register if there is one. reg address of 0xFF indicates there is none
        }
        i2c_rep_start((address << 1)| I2C_READ); //Address of the device in read mode
        LowerByte = i2c_safe_readNak(); // Get the byte, sending NACK
        i2c_stop();
        return LowerByte;
    }
    else {
        return I2C_SAFE_READ_ERROR_CODE;
    }
   
}/*i2c_safe_read_word*/


/**
    @brief      Read 16 bit word from I2C device, given address and register
    @param      address 7 bit I2C address
    @param      reg     Register address. 0xFF is for no register for dumb device
    @retval     registervalue 16 bit word
    @retval     I2C_SAFE_READ_ERROR_CODE if read fails
*/
uint16_t i2c_safe_read_sixteen(uint8_t address, uint8_t reg){
    uint8_t UpperByte;
    uint8_t LowerByte;
    if(!i2c_start((address << 1)| I2C_WRITE)){ //Address of the device in write mode
        if (reg != 0xFF) {
            i2c_write(reg); //Set the pointer register if there is one. reg address of 0xFF indicates there is none
        }
        i2c_rep_start((address << 1)| I2C_READ); //Address of the device in read mode
        UpperByte = i2c_safe_readAck(); // Get the first byte sending ACK
        LowerByte = i2c_safe_readNak(); // Get the second byte, sending NACK
        i2c_stop();    
        return ((UpperByte<<8)|LowerByte); // Return the two bytes as one 16 bit word.
    }
    
    else {
        return I2C_SAFE_READ_ERROR_CODE;
    }
}/*i2c_safe_read_sixteen*/


/**
    @brief      Write an 8 bit word to I2C device, given address and register
    @param      address 7 bit I2C address
    @param      reg     Register address
    @param      data    8 bit uint
    @return     error   Error code as 1, device failed to acknowledge, or <write address><start write data word>
*/
uint8_t i2c_safe_write_word(uint8_t address, uint8_t reg, uint8_t data){
    if (!i2c_start((address << 1)| I2C_WRITE)){
       uint8_t error = (i2c_write(reg) <<1); //Set the pointer register
        error |= (i2c_write(data)); //Write data to device
        i2c_stop();
        return error;
    }
    else {
        return 1;
    }
//     uint8_t error = (i2c_start((address << 1)| I2C_WRITE)<<2); //Address of the device in write mode
}/*i2c_safe_write_word*/


/**
    @brief      write 16 bit word to I2C device, given address and register 
    @param      address 7 bit I2C address                                         *
    @param      reg     Register address
    @param      data    16 bit uint
    @return     error   Error code as 1, device not acknowledge, or <write adress><write upper data word><write lower data word>
    @retval     0       All good
*/
uint8_t i2c_safe_write_sixteen(uint8_t address, uint8_t reg, uint16_t data){
    uint8_t UpperDataByte = (data >> 8);
    uint8_t LowerDataByte = data & 0xFF;
    uint8_t error;
    if (!i2c_start((address << 1)| I2C_WRITE)){
        error |= (i2c_write(reg)<<2); //Set the pointer register
        error |= (i2c_write(UpperDataByte)<<1); //Write data to device
        error |= (i2c_write(LowerDataByte)); //Write data to device
        i2c_stop();
        return error;
    }
    else {
        return 1;
    }
}/*i2c_safe_write_sixteen*/


/**
    @brief      Scan the I2C bus for acknowledging devices in read mode
    @param      start_addr 7 bit I2C address to start scanning at (highest value)
    @param      stop_addr  7 bit I2C address at which to stop scanning
    @return     address    7 bit I2C address of acknowledging device
    TODO: Place these in an array (of unknown size) to remove dependency on usart.h
*/
void i2c_safe_read_scan_bus(uint8_t start_addr, uint8_t stop_addr){
    while ( start_addr <= stop_addr){
        if (!i2c_start((start_addr<<1)|I2C_READ)){
            send_uint16(start_addr);
            send_newline();
        }
        start_addr++;
    }
}/*i2c_safe_read_scan_bus*/


/**
    @brief      Scan the I2C bus for acknowledging devices in write mode
    @param      start_addr 7 bit I2C address to start scanning at (highest value)
    @param      stop_addr  7 bit I2C address at which to stop scanning
    @return     address    7 bit I2C address of acknowledging device
    TODO: Place these in an array (of unknown size) to remove dependency on usart.h
*/
void i2c_safe_write_scan_bus(uint8_t start_addr, uint8_t stop_addr){
    while ( start_addr <= stop_addr){
        if (!i2c_start((start_addr<<1)|I2C_WRITE)){
            send_uint16(start_addr);
            send_newline();
        }
        start_addr++;
    }
}/*i2c_safe_write_scan_bus*/
