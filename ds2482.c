/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file ds2482.c
 *
 * @brief DS2482 I2C to 1wire bus device driver/high level commands
 *
 * - Compiler:          gcc-avr
 * - Project:           AVR-lib
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: October 2015
 *****************************************************************************/ 
#include "ds2482.h"
#include "usart.h"

#define PTR_STATUS 0xf0
#define PTR_READ 0xe1
#define PTR_CONFIG 0xc3
#define WIRE_RESET 0xb4
#define WIRE_READ 0x96
#define WIRE_SINGLE_BIT 0x87
#define WIRE_WRITE 0xa5
#define WIRE_TRIPLET 0x78

uint8_t mTimeout;
uint8_t searchAddress[8];
uint8_t searchLastDisrepancy;
uint8_t searchExhausted;

/**
    @brief      Reset the DS2482 device at the given I2C address
    @param      address 7 bit I2C address (in decimal)
    @return     status  1 success, 0 failure
*/
uint8_t ds2482_reset(uint8_t address) {
    return i2c_safe_write_reg(address,0);
}

//BEGIN Commands without i2c_stop()
uint8_t ds2482_setReadPtr(uint8_t address, uint8_t readPtr)
{
    if(!i2c_start((address << 1)| I2C_WRITE)){ 
        i2c_write(readPtr);
        return 0;
    }
    else {
        return 1;
    }
}

uint8_t ds2482_readByte(uint8_t address) {
    i2c_rep_start((address << 1)| I2C_READ);
    return i2c_safe_readNak();
}

uint8_t ds2482_wireReadStatus(uint8_t address, bool setPtr)
{
    if (setPtr)
        ds2482_setReadPtr(address, PTR_STATUS);
    
    return ds2482_readByte(address);
}

uint8_t ds2482_busyWait(uint8_t address, bool setReadPtr)
{
    uint8_t status;
    int loopCount = 1000;
    while((status = ds2482_wireReadStatus(address, setReadPtr)) & DS2482_STATUS_BUSY)
    {
        if (--loopCount <= 0)
        {
            mTimeout = 1;
            break;
        }
        _delay_ms(20);
    }
    return status;
}
//END non-stopping commands */

//BEGIN: Higher level function
/** 
 * @brief ds2482_configure, configure the DS2483 mode
 * 
 * @param addres the I2C address of the DS2482 device
 * @param config 
 * @return 1 success, 0 failure of some kind (usually no device)
 **/
uint8_t ds2482_configure(uint8_t address, uint8_t config)
{
    ds2482_busyWait(address, true);
    i2c_safe_write_word(address, 0xd2, config | (~config)<<4);

    return ds2482_readByte(address) == config ? 1 : 0;
}

/** 
 * ds2482_wireReset
 * @brief reset the one wire (OW) bus
 * 
 * @param addres the I2C address of the DS2482 device
 * @return 1 success, 0 failure of some kind (usually no device)
 **/
uint8_t ds2482_wireReset(uint8_t address) {
    ds2482_busyWait(address,true);
    i2c_safe_write_reg(address, WIRE_RESET);
    
    uint8_t status = ds2482_busyWait(address, false);
    
    // Original implementation returned true for 1, and false for 0
    return status & DS2482_STATUS_PPD;
}

/** 
 * ds2482_wireWriteByte
 * @brief Write a byte to the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @param byte The byte to write to the OW bus
 * @return 1 success, 0 failure
 **/
uint8_t ds2482_wireWriteByte(uint8_t address, uint8_t byte)
{
    ds2482_busyWait(address, true);
    return i2c_safe_write_word(address, WIRE_WRITE, byte);
}

/** 
 * ds2482_wireWriteBit
 * @brief Write a single bit to the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @param bit The bit to write to the OW bus
 * @return 1 success, 0 failure
 **/
uint8_t ds2482_wireWriteBit(uint8_t address, uint8_t bit)
{
    ds2482_busyWait(address, true);
    return i2c_safe_write_word(address, WIRE_SINGLE_BIT, bit ? 0x80 : 0);
}

/** 
 * ds2482_wireReadByte
 * @brief Read a byte from the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @return byte from OW bus or I2C_SAFE_READ_ERROR_CODE if error
 **/
uint8_t ds2482_wireReadByte(uint8_t address)
{
    ds2482_busyWait(address, true);
    i2c_safe_write_reg(address, WIRE_READ);
    ds2482_busyWait(address, false);
    return i2c_safe_read_word(address, PTR_READ);
}

/** 
 * ds2482_wireReadBit
 * @brief Read a bit from the OW bus
 * 
 * @param addres The I2C address of the DS2482 device
 * @return bit from OW bus
 **/
uint8_t ds2482_wireReadBit(uint8_t address)
{
    ds2482_wireWriteBit(address, 1);
    uint8_t status = ds2482_busyWait(address, true);
    return status & DS2482_STATUS_SBR ? 1 : 0;
}

void ds2482_wireSkip(uint8_t address)
{
    ds2482_wireWriteBit(address, 0xcc);
}

/** 
 * ds2482_wireSelect
 * @brief Select the OW device
 * 
 * @param addres The I2C address of the DS2482 device
 * @param rom[8] The rom address of the OW device
 **/
void ds2482_wireSelect(uint8_t address, uint8_t *rom)
{
    ds2482_wireWriteByte(address, 0x55);
    for (int i=0;i<8;i++)
        ds2482_wireWriteByte(address, rom[i]);
}

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
uint8_t ds2482_wireSearch(uint8_t address, uint8_t *newAddr)
{
    uint8_t i;
    uint8_t direction;
    uint8_t last_zero=0;
    
    if (searchExhausted) 
        return 0;
    
    if (!ds2482_wireReset(address)) 
        return 0;

    ds2482_busyWait(address, true);
    ds2482_wireWriteByte(address, 0xf0);
    
    for(i=1;i<65;i++) 
    {
        int romByte = (i-1)>>3;
        int romBit = 1<<((i-1)&7);
        
        if (i < searchLastDisrepancy)
            direction = searchAddress[romByte] & romBit;
        else
            direction = i == searchLastDisrepancy;
        
        ds2482_busyWait(address, false);
        i2c_safe_write_word(address,WIRE_TRIPLET, direction ? 0x80 : 0);
        
        uint8_t status = ds2482_busyWait(address, false);
        
        uint8_t id = status & DS2482_STATUS_SBR;
        uint8_t comp_id = status & DS2482_STATUS_TSB;
        direction = status & DS2482_STATUS_DIR;
        
        if (id && comp_id)
            return 0;
        else
        { 
            if (!id && !comp_id && !direction)
                last_zero = i;
        }
        
        if (direction)
            searchAddress[romByte] |= romBit;
        else
            searchAddress[romByte] &= (uint8_t)~romBit;
    }

    searchLastDisrepancy = last_zero;

    if (last_zero == 0) 
        searchExhausted = 1;
    
    for (i=0;i<8;i++) 
        newAddr[i] = searchAddress[i];
    
    return 1;  
}

uint8_t ds2482_wireReset_verbose(uint8_t address) {
    ds2482_busyWait(address,true);
    i2c_safe_write_reg(address, WIRE_RESET);
    
    uint8_t status = ds2482_busyWait(address, false);
    send_string_p(PSTR("busyWait status: "));
    send_uint8(status);
    send_newline();
    
    // Original implementation returned true for 1, and false for 0
    return status & DS2482_STATUS_PPD;
}

uint8_t ds2482_wireSearch_verbose(uint8_t address, uint8_t *newAddr)
{
    uint8_t i;
    uint8_t direction;
    uint8_t last_zero=0;
    
    send_string_p(PSTR("Running wire search\n\r"));
    if (searchExhausted)
    {
        send_string_p(PSTR("Search Exhausted\n\r"));
        return 0;
    }
    
    // Have to reverse test in this implementation.
    if (!ds2482_wireReset(address))
    {
        send_string_p(PSTR("Wire reset error\n\r"));
        return 0;
    }
    
    send_string_p(PSTR("Initial buysWait status: "));
    send_uint8(ds2482_busyWait(address, true));
    send_string_p(PSTR(", wireWriteByte status (reset): "));
    ds2482_wireWriteByte(address, 0xf0);
    send_newline();
//     ds2482_busyWait(address, true);
//     ds2482_wireWriteByte(address, 0xf0);
    
    for(i=1;i<65;i++) 
    {
        int romByte = (i-1)>>3;
        int romBit = 1<<((i-1)&7);
        
        send_string_p(PSTR("i: "));
        send_uint8(i);
        send_string_p(PSTR(", direction: "));
        send_uint8(direction);
        send_string_p(PSTR(", romByte: "));
        send_uint8(romByte);
        send_string_p(PSTR(", romBit: "));
        send_uint8(romBit);
        send_newline();
        
        if (i < searchLastDisrepancy)
            direction = searchAddress[romByte] & romBit;
        else
            direction = i == searchLastDisrepancy;
        
        send_string_p(PSTR("direction is now: "));
        send_uint8(direction);
        send_newline();
        
        send_string_p(PSTR("First buysWait status: "));
        send_uint8(ds2482_busyWait(address, false));
        send_string_p(PSTR(". writing word: "));
        send_uint8(( direction ? 0x80 : 0));
        send_string_p(PSTR(", with status: "));
        send_uint8(i2c_safe_write_word(address,WIRE_TRIPLET,( direction ? 0x80 : 0)));
        send_newline();
        
//         ds2482_busyWait(address, false);
//         i2c_safe_write_word(address,WIRE_TRIPLET,( direction ? 0x80 : 0));
        
        uint8_t status = ds2482_busyWait(address, false);
        
        uint8_t id = status & DS2482_STATUS_SBR;
        uint8_t comp_id = status & DS2482_STATUS_TSB;
        direction = status & DS2482_STATUS_DIR;
        
        send_string_p(PSTR("Second buysWait status: "));
        send_uint8(status);
        send_string_p(PSTR(". id: "));
        send_uint8(id);
        send_string_p(PSTR(", comp_id: "));
        send_uint8(comp_id);
        send_string_p(PSTR(", direction: "));
        send_uint8(direction);
        send_newline();
        
        if (id && comp_id)
        {
            send_string_p(PSTR(" Return for some reason\n\r"));
            return 0;
        }
        else
        { 
            if (!id && !comp_id && !direction)
                last_zero = i;
        }
        
        if (direction)
            searchAddress[romByte] |= romBit;
        else
            searchAddress[romByte] &= (uint8_t)~romBit;
        
        send_string_p(PSTR(" return addresses: "));
        for (i=0;i<8;i++) 
            send_uint8(searchAddress[i]);
        send_newline();
    }

    searchLastDisrepancy = last_zero;

    if (last_zero == 0) 
        searchExhausted = 1;
    
    for (i=0;i<8;i++) 
        newAddr[i] = searchAddress[i];
    
    send_string_p(PSTR("Finished something, return addresses: "));
    for (i=0;i<8;i++) 
        send_uint8(searchAddress[i]);
    return 1;  
}


uint8_t ds2482_read_word(uint8_t address, uint8_t reg) {
    return i2c_safe_read_word(address, reg);
}
//END: Higher level functions