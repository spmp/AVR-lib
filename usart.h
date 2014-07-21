#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>
# include <avr/pgmspace.h>

#ifndef BAUDE_RATE
#define BAUDE_RATE 38400
#endif

#ifndef BUF_SIZE 20
#define BUF_SIZE 20
#endif

#ifndef MAX_LINE_LEN
#define MAX_LINE_LEN 20
#endif

/* Resources: 	USART 
 * 				Rx0, Tx1 pins on Arduino
 */

void init_usart(uint16_t baude, uint32_t fcpu);

/* write a byte as given to the usart */
void send_char(uint8_t byte);

/* write a 16 bit integer to the usart in ascii decimal */
void send_uint16(uint16_t num);

/* write a 32 bit integer to the usart in ascii decimal */
void send_uint32(uint32_t num);

/* write half of a 32 bit integer to the usart in ascii decimal
 *  Similar to send_uint16 but will not truncate at 2^16*/
void send_uint32_half(uint32_t num);

/* write a null-terminated string to the usart */
void send_string(const char *s);

/* write a null-terminated string to the usart from PGM space */
void send_string_p(const char *s);

void send_newline();

/* Serial buffer size */
int num_in_serial_buffer();

/* Build line buffer from serial buffer, Check serial buffer for newline
 * and run line handler if so.*/
void handle_single_char_from_serial();

/* Set function to call to handle the char string from serial */
void usart_set_handle_char_string_from_serial(void (* handlecharstring)(const char*));
