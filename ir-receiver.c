#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <string.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "ir-receiver.h"

#define BAUD SERIAL_BAUD
#define BAUD_TOL 3   /* 115200 @ 16 MHz with U2X is 2.1% off; fine for the 16U2 */
#include <util/setbaud.h>

/* ---------------------------------------------------------------- LED */

void led_init(void)
{
    LED_DDR |= _BV(LED_PIN);
    LED_PORT &= ~_BV(LED_PIN);
}

void led_on(void)
{
    LED_PORT |= _BV(LED_PIN);
}

void led_off(void)
{
    LED_PORT &= ~_BV(LED_PIN);
}

void led_blink(void)
{
    led_on();
    _delay_ms(BLINK_MS);
    led_off();
}

/* ------------------------------------------------------------- Serial */

void serial_init(void)
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif
    UCSR0B = _BV(TXEN0);                 /* transmit only */
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);  /* 8N1 */
}

static void serial_putc(char c)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

static void serial_write(const char *s)
{
    while (*s)
        serial_putc(*s++);
}

/* Send msg over USART0 followed by CRLF. Blocks until queued. */
void serial_out(const char *msg)
{
    serial_write(msg);
    serial_putc('\r');
    serial_putc('\n');
}

/* ------------------------------------------------------------ Strings */

static char *put_str(char *p, const char *s)
{
    while (*s)
        *p++ = *s++;
    return p;
}

/* Append ndigits upper-case hex digits of value. */
static char *put_digits(char *p, uint32_t value, uint8_t ndigits)
{
    static const char digits[] = "0123456789ABCDEF";

    for (int8_t i = ndigits - 1; i >= 0; i--) {
        p[i] = digits[value & 0xF];
        value >>= 4;
    }
    return p + ndigits;
}

/* Append "0x" followed by ndigits upper-case hex digits of value. */
static char *put_hex(char *p, uint32_t value, uint8_t ndigits)
{
    *p++ = '0';
    *p++ = 'x';
    return put_digits(p, value, ndigits);
}

static uint8_t reverse_bits(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

/* ---------------------------------------------------------- IR timing */
/*
 * Typical IR receiver modules (AX-1838HS, VS1838B, TSOP38xx) output LOW
 * while a carrier burst ("mark") is present and HIGH otherwise ("space").
 *
 * NEC and Kaseikyo are pulse-distance coded:
 *   header mark, header space, N x (bit mark + short/long space), stop mark
 * Bits are sent LSB first. The protocol is chosen from the header mark
 * length, then the rest of the frame is timed against that protocol.
 * RC-5 is bi-phase coded and has its own decoder (see below).
 *
 *            header         bit mark   "0" space   "1" space   bits
 *   NEC      9000/4500 us   560 us     560 us      1690 us     32
 *   Kaseikyo 3456/1728 us   432 us     432 us      1296 us     48
 *
 * NEC repeat frames (9000/2250 us header, sent while a key is held) are
 * ignored. Kaseikyo resends the full frame; see IR_REPEAT_MS.
 *
 * Timer1 runs free at F_CPU/64 = 4 us per tick.
 */

#define US(t)  ((uint16_t)((t) / 4UL))

struct ir_timing {
    uint16_t hdr_mark_lo,  hdr_mark_hi;
    uint16_t hdr_space_lo, hdr_space_hi;
    uint16_t mark_lo,      mark_hi;
    uint16_t space_lo,     space_hi;
    uint16_t one_threshold;          /* space longer than this is a "1" */
    uint8_t  nbits;
};

static const struct ir_timing timings[] = {
    [IR_PROTO_NEC] = {
        US(8000), US(10000), US(4000), US(5000),
        US(300),  US(900),   US(300),  US(2100), US(1100), 32,
    },
    [IR_PROTO_KASEIKYO] = {
        US(3000), US(4000),  US(1400), US(2100),
        US(250),  US(700),   US(250),  US(1700), US(860),  48,
    },
};

#define IR_NPROTO (sizeof(timings) / sizeof(timings[0]))

static inline bool between(uint16_t d, uint16_t lo, uint16_t hi)
{
    return d >= lo && d <= hi;
}

/* ------------------------------------------------------ IR validation */

static bool nec_valid(const uint8_t *f)
{
    /* Command byte must be followed by its inverse. The address may be
     * 8-bit + inverse or 16-bit ("extended NEC"), so it is not checked. */
    return (uint8_t)(f[2] ^ f[3]) == 0xFF;
}

static bool kaseikyo_valid(const uint8_t *f)
{
    uint8_t vparity = f[0] ^ f[1];

    vparity = (vparity ^ (vparity >> 4)) & 0x0F;
    return (f[2] & 0x0F) == vparity                /* vendor parity */
        && f[5] == (uint8_t)(f[2] ^ f[3] ^ f[4]);  /* data parity   */
}

static bool frame_valid(enum ir_protocol proto, const uint8_t *f)
{
    switch (proto) {
    case IR_PROTO_NEC:      return nec_valid(f);
    case IR_PROTO_KASEIKYO: return kaseikyo_valid(f);
    }
    return false;
}

/* ------------------------------------------------------------ IR core */

enum ir_state {
    IR_IDLE, IR_LEAD_MARK, IR_LEAD_SPACE, IR_BIT_MARK, IR_BIT_SPACE,
    IR_RC5, IR_RC5_DONE, IR_NEC_RPT
};

static volatile struct ir_code ir_result;
static volatile bool           ir_ready;
static volatile uint16_t       timer_ovf;   /* upper 16 bits of tick count */

void ir_init(void)
{
    IR_DDR  &= ~_BV(IR_PIN);   /* input */
    IR_PORT |=  _BV(IR_PIN);   /* pull-up, module idles high */

    TCCR1A = 0;
    TCCR1B = _BV(CS11) | _BV(CS10);   /* normal mode, clk/64 */
    TIMSK1 = _BV(TOIE1);

    PCMSK0 |= _BV(IR_PCINT);
    PCICR  |= _BV(PCIE0);
}

#if IR_DEBUG
/*
 * Raw capture: every edge pushes the duration of the interval that just
 * ended. main() prints them as IRremote-style "+mark -space" in us.
 */
#define RAW_SIZE 128   /* power of two; > one Kaseikyo frame (100 edges) */

struct raw_edge {
    uint16_t ticks;
    bool     mark;     /* true: a mark (IR on) just ended */
};

static volatile struct raw_edge raw_buf[RAW_SIZE];
static volatile uint8_t         raw_head, raw_tail;
static volatile uint16_t        raw_dropped;

static void raw_push(uint16_t ticks, bool mark)
{
    uint8_t next = (raw_head + 1) & (RAW_SIZE - 1);

    if (next == raw_tail) {
        raw_dropped++;
        return;
    }
    raw_buf[raw_head].ticks = ticks;
    raw_buf[raw_head].mark  = mark;
    raw_head = next;
}

static bool raw_pop(struct raw_edge *e)
{
    bool ok = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (raw_tail != raw_head) {
            e->ticks = raw_buf[raw_tail].ticks;
            e->mark  = raw_buf[raw_tail].mark;
            raw_tail = (raw_tail + 1) & (RAW_SIZE - 1);
            ok = true;
        }
    }
    return ok;
}

static char *put_dec(char *p, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0;

    do {
        tmp[n++] = '0' + v % 10;
        v /= 10;
    } while (v);
    while (n)
        *p++ = tmp[--n];
    return p;
}

/* Print all pending raw edges. A long space ends the current line. */
static void raw_print(void)
{
    struct raw_edge e;
    char buf[16];

    while (raw_pop(&e)) {
        uint32_t us = (uint32_t)e.ticks * 4;
        char *p = buf;

        if (!e.mark && us > 20000) {
            /* Idle gap before a frame (may have wrapped past 262 ms). */
            serial_out("");
            continue;
        }
        *p++ = ' ';
        *p++ = e.mark ? '+' : '-';
        p = put_dec(p, us);
        *p = '\0';
        serial_write(buf);
    }
    if (raw_dropped) {
        serial_out("");
        serial_out("[raw buffer overflow]");
        raw_dropped = 0;
    }
}
#endif /* IR_DEBUG */

ISR(TIMER1_OVF_vect)
{
    timer_ovf++;
}

/* 32-bit tick count. Call with interrupts disabled. */
static uint32_t ticks_now(uint16_t lo)
{
    uint16_t hi = timer_ovf;

    /* Overflow happened but its ISR has not run yet. */
    if ((TIFR1 & _BV(TOV1)) && lo < 0x8000)
        hi++;
    return ((uint32_t)hi << 16) | lo;
}

/* Publish a complete, valid frame. Repeats of the last frame are flagged. */
static void ir_publish(enum ir_protocol proto, const uint8_t *frame, uint8_t len, uint16_t lo)
{
    static struct ir_code last;
    static uint32_t       last_time;
    static bool           have_last;

    uint32_t now = ticks_now(lo);
    bool     repeat = have_last
                   && last.protocol == proto
                   && memcmp(last.data, frame, len) == 0
                   && now - last_time < (uint32_t)IR_REPEAT_MS * 1000 / 4;

    last.protocol = proto;
    last.len      = len;
    memcpy(last.data, frame, len);
    last_time = now;
    have_last = true;

    if (repeat && !IR_SHOW_REPEATS)
        return;

    ir_result.protocol = proto;
    ir_result.len      = len;
    ir_result.repeat   = repeat;
    for (uint8_t i = 0; i < len; i++)
        ir_result.data[i] = frame[i];
    ir_ready = true;
}

/* NEC repeat code: 9000/2250 us header, no data. It repeats the last key. */
static void ir_publish_nec_repeat(void)
{
    if (!IR_SHOW_REPEATS)
        return;
    ir_result.protocol = IR_PROTO_NEC;
    ir_result.len      = 0;
    ir_result.repeat   = true;
    ir_ready           = true;
}

/* ---------------------------------------------------------- RC-5 */
/*
 * Philips RC-5: 14 bits, bi-phase (Manchester) coded, MSB first, 36 kHz.
 * Each bit is two 889 us half-bits. A "1" is space-then-mark and a "0" is
 * mark-then-space, so the level of the second half-bit is the bit value.
 * Frame: S1 S2 T A4..A0 C5..C0
 *   S1 start bit (always 1), S2 = inverted command bit 6 (extended RC-5),
 *   T  toggle (flips on each new key press), A address, C command.
 *
 * A run of one level lasts 1 or 2 half-bits. Each run is checked when it
 * ends, and a run that starts on a second half-bit carries a bit.
 */
#define RC5_HALF_MIN  US(600)
#define RC5_HALF_MAX  US(1200)
#define RC5_FULL_MIN  US(1400)
#define RC5_FULL_MAX  US(2200)

static uint16_t rc5_word;
static uint8_t  rc5_nbits;
static uint8_t  rc5_phase;   /* 1: the current run starts on a second half-bit */

/* Half-bits in a run of d ticks (1 or 2); 0 if not an RC-5 run. */
static uint8_t rc5_halves(uint16_t d)
{
    if (between(d, RC5_HALF_MIN, RC5_HALF_MAX))
        return 1;
    if (between(d, RC5_FULL_MIN, RC5_FULL_MAX))
        return 2;
    return 0;
}

/* Close the run that just ended (length d), then start a run at level low.
 * Returns 1 when 14 bits are complete, -1 on a framing error, else 0. */
static int rc5_edge(bool low, uint16_t d)
{
    uint8_t n = rc5_halves(d);

    /* A 2-half run must begin on a second half-bit, otherwise a level
     * would last across a whole bit, which RC-5 never does. */
    if (n == 0 || (n == 2 && rc5_phase == 0))
        return -1;
    rc5_phase = (rc5_phase + n) & 1;

    if (rc5_phase == 1) {
        rc5_word = (rc5_word << 1) | (low ? 1 : 0);
        if (++rc5_nbits == 14)
            return 1;
    }
    return 0;
}

/* Called on the rising edge that ends the start bit S1's mark. */
static void rc5_begin(void)
{
    rc5_word  = 1;   /* S1 */
    rc5_nbits = 1;
    rc5_phase = 1;   /* the mark ending now began on S1's second half-bit */
}

/* Advance the RC-5 decoder by one run. Returns the next ir_state. */
static enum ir_state rc5_step(bool low, uint16_t d, uint16_t now)
{
    uint8_t f[3];
    int     r = rc5_edge(low, d);

    if (r < 0)
        return IR_IDLE;
    if (r == 0)
        return IR_RC5;

    f[0] = (rc5_word >> 6) & 0x1F;                                /* address */
    f[1] = (rc5_word & 0x3F) | ((rc5_word & 0x1000) ? 0 : 0x40); /* command */
    f[2] = (rc5_word >> 11) & 1;                                  /* toggle  */
    ir_publish(IR_PROTO_RC5, f, 3, now);
    return IR_RC5_DONE;
}

#define NEC_RPT_SPACE_LO  US(2000)   /* NEC repeat header space: 2250 us */
#define NEC_RPT_SPACE_HI  US(2500)

ISR(PCINT0_vect)
{
    static enum ir_state    state = IR_IDLE;
    static enum ir_protocol proto;
    static uint16_t         last;
    static uint8_t          frame[IR_MAX_BYTES];
    static uint8_t          bits;

    uint16_t now = TCNT1;
    uint16_t d   = now - last;
    bool     low = !(IR_PINR & _BV(IR_PIN));   /* true: a mark just began */
    const struct ir_timing *t = &timings[proto];

    last = now;

#if IR_DEBUG
    /* LED mirrors the IR carrier: on while a mark is being received. */
    if (low)
        led_on();
    else
        led_off();
    raw_push(d, !low);
#endif

    if (state == IR_RC5) {
        state = rc5_step(low, d, now);
        if (state != IR_IDLE)
            return;
    }
    if (state == IR_RC5_DONE) {
        if (d <= RC5_FULL_MAX)
            return;          /* still in the frame's idle tail */
        state = IR_IDLE;     /* fall through: this edge may start a frame */
    }

    if (low) {
        /* Falling edge: a space just ended. */
        switch (state) {
        case IR_LEAD_SPACE:
            if (between(d, t->hdr_space_lo, t->hdr_space_hi)) {
                memset(frame, 0, sizeof(frame));
                bits  = 0;
                state = IR_BIT_MARK;
                return;
            }
            if (proto == IR_PROTO_NEC && between(d, NEC_RPT_SPACE_LO, NEC_RPT_SPACE_HI)) {
                state = IR_NEC_RPT;   /* repeat code: confirmed by its stop mark */
                return;
            }
            break;
        case IR_BIT_SPACE:
            if (between(d, t->space_lo, t->space_hi)) {
                if (d > t->one_threshold)
                    frame[bits >> 3] |= 1 << (bits & 7);
                bits++;
                state = IR_BIT_MARK;
                return;
            }
            break;
        default:
            break;
        }
        /* Anything unexpected may be the start of a new frame. */
        state = IR_LEAD_MARK;
    } else {
        /* Rising edge: a mark just ended. */
        switch (state) {
        case IR_LEAD_MARK:
            state = IR_IDLE;
            for (uint8_t p = 0; p < IR_NPROTO; p++) {
                if (between(d, timings[p].hdr_mark_lo, timings[p].hdr_mark_hi)) {
                    proto = (enum ir_protocol)p;
                    state = IR_LEAD_SPACE;
                    break;
                }
            }
            if (state == IR_IDLE && rc5_halves(d)) {
                rc5_begin();
                state = rc5_step(false, d, now);
            }
            break;
        case IR_NEC_RPT:
            if (between(d, t->mark_lo, t->mark_hi))
                ir_publish_nec_repeat();
            state = IR_IDLE;
            break;
        case IR_BIT_MARK:
            if (!between(d, t->mark_lo, t->mark_hi)) {
                state = IR_IDLE;
            } else if (bits == t->nbits) {
                /* Stop mark: frame complete. */
                if (frame_valid(proto, frame))
                    ir_publish(proto, frame, timings[proto].nbits / 8, now);
                state = IR_IDLE;
            } else {
                state = IR_BIT_SPACE;
            }
            break;
        default:
            state = IR_IDLE;
            break;
        }
    }
}

/* Fetch a decoded code if one is waiting. */
bool ir_read(struct ir_code *code)
{
    bool ok = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (ir_ready) {
            code->protocol = ir_result.protocol;
            code->len      = ir_result.len;
            code->repeat   = ir_result.repeat;
            for (uint8_t i = 0; i < ir_result.len; i++)
                code->data[i] = ir_result.data[i];
            ir_ready = false;
            ok       = true;
        }
    }
    return ok;
}

/*
 * Format a code as one line (buf >= 72 bytes):
 *   NEC addr=0x00 cmd=0x46 raw=0x00FF629D
 *   PANASONIC addr=0x... cmd=0x.. raw=0x0220........
 *   KASEIKYO vendor=0x0003 addr=0x... cmd=0x.. raw=0x...
 *   RC5 addr=0x05 cmd=0x0C toggle=0 raw=0x314C
 *
 * addr/cmd follow the IRremote library conventions so codes can be
 * compared against published tables.
 */
void ir_format(const struct ir_code *code, char *buf)
{
    const uint8_t *f = code->data;
    char *p = buf;

    switch (code->protocol) {
    case IR_PROTO_NEC: {
        if (code->len == 0) {
            p = put_str(p, "NEC repeat code");
            break;
        }
        /* raw: classic MSB-first value, e.g. 0x00FF629D */
        uint32_t raw = (uint32_t)reverse_bits(f[0]) << 24
                     | (uint32_t)reverse_bits(f[1]) << 16
                     | (uint32_t)reverse_bits(f[2]) << 8
                     | reverse_bits(f[3]);
        bool extended = (uint8_t)(f[0] ^ f[1]) != 0xFF;

        p = put_str(p, "NEC addr=");
        p = extended ? put_hex(p, f[0] | (uint16_t)f[1] << 8, 4)
                     : put_hex(p, f[0], 2);
        p = put_str(p, " cmd=");
        p = put_hex(p, f[2], 2);
        p = put_str(p, " raw=");
        p = put_hex(p, raw, 8);
        break;
    }
    case IR_PROTO_RC5: {
        /* raw: the 14-bit frame as received, S1 S2 T A4..A0 C5..C0 */
        uint16_t raw = (1u << 13)                          /* S1 */
                     | (f[1] & 0x40 ? 0 : 1u << 12)        /* S2 = !cmd bit 6 */
                     | (uint16_t)f[2] << 11                /* toggle */
                     | (uint16_t)f[0] << 6                 /* address */
                     | (f[1] & 0x3F);                      /* command */

        p = put_str(p, "RC5 addr=");
        p = put_hex(p, f[0], 2);
        p = put_str(p, " cmd=");
        p = put_hex(p, f[1], 2);
        p = put_str(p, " toggle=");
        *p++ = '0' + f[2];
        p = put_str(p, " raw=");
        p = put_hex(p, raw, 4);
        break;
    }
    case IR_PROTO_KASEIKYO: {
        uint16_t vendor = f[0] | (uint16_t)f[1] << 8;
        uint16_t addr   = (f[2] >> 4) | (uint16_t)f[3] << 4;

        if (vendor == 0x2002) {
            p = put_str(p, "PANASONIC");
        } else {
            p = put_str(p, "KASEIKYO vendor=");
            p = put_hex(p, vendor, 4);
        }
        p = put_str(p, " addr=");
        p = put_hex(p, addr, 3);
        p = put_str(p, " cmd=");
        p = put_hex(p, f[4], 2);
        p = put_str(p, " raw=0x");
        for (uint8_t i = 0; i < code->len; i++)
            p = put_digits(p, f[i], 2);
        break;
    }
    }
    *p = '\0';
}

/* --------------------------------------------------------------- main */

/* Overrides the Arduino core main(); the core's main.o is not linked in. */
int main(void)
{
    struct ir_code code, last;
    bool           have_last = false;
    char           buf[72], last_buf[72];

    led_init();
    serial_init();
    ir_init();
    set_sleep_mode(SLEEP_MODE_IDLE);   /* Timer1 keeps running */
    sei();

#if IR_DEBUG
    serial_out("ir-receiver ready (debug: raw timings in us, +mark -space)");
#else
    serial_out("ir-receiver ready");
#endif
    serial_write("IR input D11 idle level: ");
    serial_out((IR_PINR & _BV(IR_PIN)) ? "HIGH (ok)" : "LOW (check wiring)");

    for (;;) {
        cli();
#if IR_DEBUG
        if (!ir_ready && raw_head == raw_tail) {
#else
        if (!ir_ready) {
#endif
            /* sei immediately before sleep: no wakeup can be missed. */
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        }
        sei();

#if IR_DEBUG
        raw_print();
#endif
        if (ir_read(&code)) {
            ir_format(&code, buf);
#if IR_DEBUG
            serial_out("");
#endif
            if (!code.repeat) {
                last      = code;
                have_last = true;
                serial_out(buf);
#if !IR_DEBUG
                led_blink();
#endif
            } else {
                /* Indented: same key as the line above it. */
                serial_write("    ");
                serial_write(buf);
                if (code.len == 0 && have_last) {
                    ir_format(&last, last_buf);
                    serial_write(" of ");
                    serial_write(last_buf);
                }
                serial_out("");
            }
        }
    }

    return 0;
}
