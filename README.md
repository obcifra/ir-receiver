# ir-receiver

Bare-metal IR receiver for the Arduino Mega 2560 (ATmega2560, 16 MHz). It decodes
IR remote frames and prints them over USB serial. Written for `avr-gcc` through
`arduino-cli`, without the Arduino core's `main()`.

## Hardware

| Signal | Mega pin | Port/bit |
|---|---|---|
| IR receiver output (AX-1838HS, VS1838B, TSOP38xx) | D11 | PB5 (PCINT5) |
| Status LED (onboard) | D13 | PB7 |

The receiver module idles high and goes low while a carrier burst is present.
Connect VCC to 5 V and GND to GND. The sketch uses the internal pull-up on D11.

Serial output is on USART0, which the Mega's USB-serial bridge carries
to the host at 115200 baud, 8N1.

## Protocol support

| Protocol | Bits | Carrier | Notes |
|---|---|---|---|
| NEC | 32 | 38 kHz | Address is 8-bit (checked against its inverse) or 16-bit "extended". Command byte must match its inverse. Repeat codes are recognized (see `REPEAT`). |
| Kaseikyo | 48 | ~37 kHz | Used by Panasonic, Denon, JVC, and others. Vendor parity and data parity are checked. Panasonic (vendor `0x2002`) is shown by name. |
| RC-5 | 14 | 36 kHz | Philips bi-phase (Manchester) code, 889 µs half-bits. Shows address, command (with extended bit 6 from S2), and toggle. |

Frames that fail validation are discarded. Protocols are chosen from the
header timing, so NEC, Kaseikyo, and RC-5 can all be received at the same time.

## Build and flash

Requires `arduino-cli` with the `arduino:avr` core and the Mega 2560 board
package installed.

| Command | What it does |
|---|---|
| `make build` | Compile to `build/` |
| `make upload` | Compile, then flash the Mega |
| `make monitor` | Open the serial monitor at `$(BAUD)` on `$(PORT)` |
| `make clean` | Remove `build/` |

Variables can be set on the command line:

| Variable | Default | Meaning |
|---|---|---|
| `PORT` | `/dev/ttyACM0` | Serial port of the Mega |
| `BAUD` | `115200` | Serial baud rate |
| `FQBN` | `arduino:avr:mega:cpu=atmega2560` | Board to build for |
| `DEBUG` | `0` | `1` enables raw timing output (see below) |
| `REPEAT` | `0` | `1` prints repeats, indented under the key they repeat (see below) |

Examples:

```sh
make upload                       # normal build
make upload REPEAT=1              # also show repeats
make upload DEBUG=1 REPEAT=1      # raw timings and repeats
make upload PORT=/dev/ttyACM1     # different port
make monitor                      # watch output
```

Changing `DEBUG` or `REPEAT` rebuilds the sketch, so you don't need `make clean`.

## Options

### `DEBUG=1`

- The LED follows the IR input: on while a mark (carrier) is being received.
- Before each decoded line, the raw pulse timings are printed, one edge per
  pulse, as `+mark -space` in microseconds. A blank line separates frames.
- A long space (over 20 ms) ends a line of timings.
- `[raw buffer overflow]` means edges were dropped because the buffer filled.
- The LED does not blink on decoded frames in this mode.

Example:

```
ir-receiver ready (debug: raw timings in us, +mark -space)
IR input D11 idle level: HIGH (ok)

 +9000 -4500 +560 -560 +560 -1690 ...
NEC addr=0x00 cmd=0x46 raw=0x00FF629D
```

Use this to check protocol timing against a new remote.

### `REPEAT=1`

By default, repeats are dropped. With `REPEAT=1`, they are printed,
indented four spaces under the key they repeat:

```
NEC addr=0x00 cmd=0x46 raw=0x00FF629D
    NEC repeat code of NEC addr=0x00 cmd=0x46 raw=0x00FF629D
    NEC repeat code of NEC addr=0x00 cmd=0x46 raw=0x00FF629D
PANASONIC addr=0x00B cmd=0x16 raw=0x0220B00016A6
    PANASONIC addr=0x00B cmd=0x16 raw=0x0220B00016A6
RC5 addr=0x05 cmd=0x0C toggle=0 raw=0x314C
    RC5 addr=0x05 cmd=0x0C toggle=0 raw=0x314C
```

How repeats appear depends on the protocol:

- **NEC** sends a short repeat code (9000/2250 µs header, no data), so the
  line says `NEC repeat code of <last key>`.
- **Kaseikyo** and **RC-5** send the full frame again, so the same line is
  printed with an indent.
- A frame counts as a repeat when it matches the previous frame and arrives
  within 150 ms (`IR_REPEAT_MS`). RC-5 frames with a flipped toggle are new presses.
- The LED blinks only on new presses.

## Output format

Each decoded frame is one line of CRLF-terminated text:

| Protocol | Example |
|---|---|
| NEC | `NEC addr=0x00 cmd=0x46 raw=0x00FF629D` |
| NEC extended | `NEC addr=0x1234 cmd=0x46 raw=...` (16-bit address) |
| Panasonic | `PANASONIC addr=0x00B cmd=0x16 raw=0x0220B00016A6` |
| Kaseikyo (other) | `KASEIKYO vendor=0x0003 addr=0x... cmd=0x.. raw=...` |
| RC-5 | `RC5 addr=0x05 cmd=0x0C toggle=0 raw=0x314C` |

Fields:

- `addr` and `cmd` follow the IRremote library's conventions, so codes can be
  compared with published tables.
- `raw` is the frame as received. NEC is the classic MSB-first 32-bit value.
  Kaseikyo is the bytes in transmit order. RC-5 is the 14-bit frame, S1 S2 T A C.
- For RC-5, `cmd` includes bit 6 from the inverted S2 bit (commands 64–127).

## Source layout

| File | Contents |
|---|---|
| `ir-receiver.h` | Pin definitions, `DEBUG`/`REPEAT` defaults, protocol enum, public API |
| `ir-receiver.c` | Pin setup, USART output, PCINT decoder for NEC/Kaseikyo/RC-5, `main()` |
| `Makefile` | Build, upload, monitor, and the `DEBUG`/`REPEAT` flags |
