SKETCH    := ir-receiver
FQBN      ?= arduino:avr:mega:cpu=atmega2560
PORT      ?= /dev/ttyACM0
BAUD      ?= 115200
DEBUG     ?= 0
BUILD_DIR := build

CFLAGS := -DSERIAL_BAUD=$(BAUD)UL -DIR_DEBUG=$(DEBUG)

.PHONY: build upload monitor clean

# Always invoke arduino-cli: it rebuilds incrementally and picks up
# changes to BAUD/DEBUG, which file timestamps would miss.
build:
	arduino-cli compile --fqbn $(FQBN) --build-property "compiler.c.extra_flags=$(CFLAGS)" --output-dir $(BUILD_DIR) .

upload: build
	arduino-cli upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_DIR) .

monitor:
	arduino-cli monitor --port $(PORT) --config baudrate=$(BAUD)

clean:
	rm -rf $(BUILD_DIR)
