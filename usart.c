#include <avr/io.h>
#include "usart.h"

void (* handleCharLine)(const char*); // void function pointer
char incoming_buffer[BUF_SIZE];
int truncating_data = 0; // hopefully this doesn't become 1!
int serial_buffer_count = 0;
int in_start = 0;

char line_buffer[MAX_LINE_LEN+1];
int num_char = 0;

/* usart enable send/recv mode: 8N1 @ 57600 */
void init_usart(uint16_t baude, uint32_t fcpu) {
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
    UBRR0 = fcpu / 16 / baude - 1;
}

void usart_set_handle_char_string_from_serial(void (* handlecharstring)(const char*)) {
    handleCharLine = handlecharstring;
}

/* write a single byte to the usart */
void send_char(uint8_t byte) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = byte;
}

/* write a 16 bit integer to the usart in ascii decimal */
void send_uint16(uint16_t num) {
    uint16_t mult = 10000;
    for (uint8_t i=0; i<5; i++) {
        uint16_t face_value = num / mult;
        send_char('0' + face_value);
        num -= face_value * mult;
        mult /= 10;
    }
}

/* write a 32 bit integer to the usart in ascii decimal */
void send_uint32(uint32_t num) {
    uint32_t mult = 1000000000;
    for (uint8_t i=0; i<10; i++) {
        uint32_t face_value = num / mult;
        send_char('0' + face_value);
        num -= face_value * mult;
        mult /= 10;
    }
}

/* write half of a 32 bit integer to the usart in ascii decimal */
void send_uint32_half(uint32_t num) {
    uint32_t mult = 10000;
    for (uint8_t i=0; i<5; i++) {
        uint32_t face_value = num / mult;
        send_char('0' + face_value);
        num -= face_value * mult;
        mult /= 10;
    }
}

/* write a null-terminated string to the usart */
void send_string(const char *s) {
    while (*s) {
        send_char(*s++);
    }
}

/* write a null-terminated string to the usart */
void send_string_p(const char *s) {
    while (pgm_read_byte(s)) {
        send_char(pgm_read_byte(s++));
    }
}

void send_newline() {
    send_string_p(PSTR("\r\n"));
//     send_char('\r');
//     send_char('\n');
}

/* Handle USART reading. So much more _fun! */
ISR (USART_RX_vect) {
    incoming_buffer[(in_start + serial_buffer_count) % BUF_SIZE] = UDR0;
    if (serial_buffer_count >= BUF_SIZE) {
        truncating_data = 1; // b-b-b-add!
    } else {
        serial_buffer_count++;
    }
}

/* blocks until chars have been received, return the last recieved byte */
char serial_read() {
    if (serial_buffer_count == 0)
        return '\0';
    char result = incoming_buffer[in_start];
    serial_buffer_count--;
    in_start = (in_start + 1) % BUF_SIZE;
    return result;
}

/* Serial buffer size */
int num_in_serial_buffer() {
    return serial_buffer_count;
}

/* Build line buffer from serial buffer, Check serial buffer for newline
 * and run line handler if so.*/
void handle_single_char_from_serial() {
    char next_char = serial_read();
    switch(next_char) {
        case '\n':
        case '\r': // This is what the terminal sends as newline
            line_buffer[num_char] = '\0'; //terminate the line_buffer
            if (num_char > 0) { // Does the line contain any information
                handleCharLine(line_buffer);
            }
            num_char = 0;
            break;
        case '\0':
            num_char = 0; // this shouldn't happen, but reset buffer anyway
            break;
        default: // save characters to line buffer
            if (num_char < MAX_LINE_LEN) // overflow check
                line_buffer[num_char++] = next_char;
            break;
        }
}

