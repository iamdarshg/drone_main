PREFIX    = arm-none-eabi
CC        = $(PREFIX)-gcc
CFLAGS    = -Wall -O2 -g -ffunction-sections -fdata-sections
LDFLAGS   = -Wl,--gc-sections -Tlink_dual.ld

M4_FLAGS  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DCORE_M4
M0_FLAGS  = -mcpu=cortex-m0 -mthumb -DCORE_M0

INCLUDES  = -Iinclude -ILPCOpen -Ifreertos/include -Ifreertos/source/include

M4_SRCS   = $(wildcard src/m4/*.c) $(wildcard freertos/source/*.c)
M0_SRCS   = $(wildcard src/m0/*.c)

M4_OBJS   = $(patsubst %.c, build/m4/%.o, $(M4_SRCS))
M0_OBJS   = $(patsubst %.c, build/m0/%.o, $(M0_SRCS))

ELF_M4    = build/m4/m4.elf
ELF_M0    = build/m0/m0.elf
BIN_M4    = build/m4/m4.bin
BIN_M0    = build/m0/m0.bin
COMBINED  = build/firmware_combined.bin

all: $(BIN_M4) $(BIN_M0) $(COMBINED)
	@echo "✅ Build complete"

build/m4/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(M4_FLAGS) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/m0/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(M0_FLAGS) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(ELF_M4): $(M4_OBJS)
	$(CC) $(M4_FLAGS) $(LDFLAGS) $^ -o $@

$(ELF_M0): $(M0_OBJS)
	$(CC) $(M0_FLAGS) $(LDFLAGS) $^ -o $@

$(BIN_M4): $(ELF_M4)
	$(PREFIX)-objcopy -O binary $< $@

$(BIN_M0): $(ELF_M0)
	$(PREFIX)-objcopy -O binary $< $@

$(COMBINED): $(BIN_M4) $(BIN_M0)
	@cat $(BIN_M4) $(BIN_M0) > $@

clean:
	rm -rf build
