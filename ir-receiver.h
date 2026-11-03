#ifndef IR_RECEIVER_H
#define IR_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

/* Onboard LED on the Mega2560 is digital pin 13 = PB7 */
#define LED_DDR   DDRB
#define LED_PORT  PORTB
#define LED_PIN   PB7

#define BLINK_MS  100

/* IR receiver output on digital pin 11 = PB5 (PCINT5) */
#define IR_DDR    DDRB
#define IR_PORT   PORTB
#define IR_PINR   PINB
#define IR_PIN    PB5
#define IR_PCINT  PCINT5

/* Identical frames closer together than this are treated as key repeats */
#define IR_REPEAT_MS 150

/* 1: LED mirrors the IR input and raw pulse timings are printed */
#ifndef IR_DEBUG
#define IR_DEBUG 0
#endif

/* 1: print auto-repeats, indented under the key they repeat (make REPEAT=1) */
#ifndef IR_SHOW_REPEATS
#define IR_SHOW_REPEATS 0
#endif

/* USART0 is wired to the USB-serial bridge on the Mega2560 */
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200UL
#endif

#define IR_MAX_BYTES 6

enum ir_protocol {
    IR_PROTO_NEC,        /* 32 bits, 38 kHz */
    IR_PROTO_KASEIKYO,   /* 48 bits, ~37 kHz: Panasonic, Denon, JVC, ... */
    IR_PROTO_RC5,        /* 14 bits, 36 kHz, bi-phase: Philips, Arcam, ... */
};

struct ir_code {
    enum ir_protocol protocol;
    uint8_t          len;                 /* bytes used in data[]; 0 for NEC repeat code */
    uint8_t          data[IR_MAX_BYTES];  /* NEC/Kaseikyo: frame bytes in transmit order;
                                             RC5: addr, cmd (bit 6 = !S2), toggle */
    bool             repeat;              /* same key as the previous code */
};

#ifdef __cplusplus
extern "C" {
#endif

void led_init(void);
void led_on(void);
void led_off(void);
void led_blink(void);

void serial_init(void);
void serial_out(const char *msg);

void ir_init(void);
bool ir_read(struct ir_code *code);
void ir_format(const struct ir_code *code, char *buf);

#ifdef __cplusplus
}
#endif

#endif /* IR_RECEIVER_H */
