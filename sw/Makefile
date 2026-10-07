# Firmware for the K-means SoC: RV32I, no C library, libgcc for the
# multiply and divide helpers (RV32I has no M extension).
PREFIX  ?= riscv64-unknown-elf-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size
BUILD   ?= ../build
WORDS   ?= 4096         # RAM size in 32-bit words (soc.sv RAM_WORDS)

CFLAGS  := -march=rv32i -mabi=ilp32 -O2 -ffreestanding -fno-builtin -mno-relax \
           -Wall -Wextra -Werror -std=c11 -ffunction-sections -fdata-sections
LDFLAGS := -nostdlib -nostartfiles -T link.ld -Wl,--gc-sections,--no-warn-rwx-segments
SRCS    := crt0.S main.c kmeans.c lib.c

all: $(BUILD)/firmware.hex

$(BUILD)/firmware.elf: $(SRCS) soc.h protocol.h kmeans.h crc16_table.h link.ld | $(BUILD)
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -lgcc -o $@
	$(SIZE) $@

$(BUILD)/firmware.hex: $(BUILD)/firmware.elf
	$(OBJCOPY) -O binary $< $(BUILD)/firmware.bin
	python3 ../tools/bin2hex.py $(BUILD)/firmware.bin $@ $(WORDS)

$(BUILD):
	mkdir -p $@

clean:
	rm -f $(BUILD)/firmware.*

.PHONY: all clean
