include ../../py/mkenv.mk

CROSS = 1

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# MicroPython feature configurations
MICROPY_ROM_TEXT_COMPRESSION ?= 1

# include py core make definitions
include $(TOP)/py/py.mk

ifeq ($(CROSS), 1)
CROSS_COMPILE ?= arm-eabi-
endif

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I../../machine/include -I../../arch/arm32/include -I../../machine/include/f1c100s
INC	+= -I../../driver/include -I../../lib

ifeq ($(CROSS), 1)
DFU =
PYDFU = 
# CFLAGS_CORTEX_M4 = 
# CFLAGS = $(INC) -Wall -Werror -std=c99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)
# LDFLAGS = -nostdlib -T stm32f405.ld -Map=$@.map --cref --gc-sections
CC	          = $(CROSS_COMPILE)gcc
AS	          = $(CROSS_COMPILE)gcc -x assembler-with-cpp
LD	          = $(CROSS_COMPILE)ld
CFLAGS		:= -g -ggdb -Wall -O3 -ffreestanding -std=c99 $(INC)
ASFLAGS		:= -g -ggdb -Wall -O3 -ffreestanding -std=gnu99 $(INC)
LDFLAGS		:= -T f1c100s.ld -nostdlib
MCFLAGS		:= -march=armv5te -mtune=arm926ej-s -mfloat-abi=soft -marm -mno-thumb-interwork
else
LD = gcc
CFLAGS = -m32 $(INC) -Wall -Werror -Wdouble-promotion -Wfloat-conversion -std=c99 $(COPT)
LDFLAGS = -m32 -Wl,-Map=$@.map,--cref -Wl,--gc-sections
endif

CSUPEROPT = -O3 # save some code space

# Tune for Debugging or Optimization
# ifeq ($(DEBUG), 1)
# CFLAGS += -O0 -ggdb
# else
# CFLAGS += -Os -DNDEBUG
# CFLAGS += -fdata-sections -ffunction-sections
# endif

SRC_C = \
	main.c \
	uart_core.c \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/mp-readline/readline.c \
	$(BUILD)/_frozen_mpy.c \

ifeq ($(CROSS), 1)
SRC_ASM = machine/start.S arch/arm32/lib/memcpy.S arch/arm32/lib/memset.S
SRC_C += \
	main.c \
	machine/sys-clock.c \
	machine/sys-dram.c \
	machine/sys-uart.c \
	machine/sys-copyself.c \
	machine/sys-spi-flash.c \
	machine/sys-mmu.c \
	machine/exception.c \
	machine/delay.c \
	lib/malloc.c \
	lib/dma.c \
	lib/printf.c \
	lib/string/strcpy.c \
	lib/string/strcat.c \
	lib/string/strchr.c \
	lib/string/strcmp.c \
	lib/string/strncmp.c \
	lib/string/strlen.c \
	lib/string/strstr.c \
	lib/string/strrchr.c \
	lib/string/strcpy.c \
	lib/string/memcmp.c \
	lib/string/memmove.c \
	driver/clk-f1c100s-pll.c \
	driver/fb-f1c100s.c \
	driver/irq_misc.c \
	driver/wdog-f1c100s.c \
	driver/uart-16550.c \
	driver/spi-f1c100s.c \
	driver/gpio-f1c100s.c \
	driver/pwm-f1c100s.c \
	driver/reset-f1c100s.c 

LIBS := -lgcc
endif

OBJ = $(PY_CORE_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o)) $(addprefix $(BUILD)/, $(SRC_ASM:.S=.o))

ifeq ($(CROSS), 1)
all: $(BUILD)/firmware.bin
else
all: $(BUILD)/firmware.elf
endif

$(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

$(BUILD)/firmware.elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(CC) $(LDFLAGS) -Wl,--cref,-Map=$@.map -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

MKSUNXI		:= tools/mksunxi/mksunxi

ifeq ($(CROSS), 1)
$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(OBJCOPY) -v -O binary $^ $@
	@echo Make header information for brom booting
	@$(MKSUNXI) $@
else
$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(Q)$(OBJCOPY) -O binary -j .isr_vector -j .text -j .data $^ $(BUILD)/firmware.bin
endif	

$(BUILD)/firmware.dfu: $(BUILD)/firmware.bin
	$(ECHO) "Create $@"
	$(Q)$(PYTHON) $(DFU) -b 0x08000000:$(BUILD)/firmware.bin $@

deploy: $(BUILD)/firmware.dfu
	$(ECHO) "Writing $< to the board"
	$(Q)$(PYTHON) $(PYDFU) -u $<

# Run emulation build on a POSIX system with suitable terminal settings
run:
	stty raw opost -echo
	build/firmware.elf
	@echo Resetting terminal...
# This sleep is useful to spot segfaults
	sleep 1
	reset

test: $(BUILD)/firmware.elf
	$(Q)/bin/echo -e "print('hello world!', list(x+1 for x in range(10)), end='eol\\\\n')\\r\\n\\x04" | $(BUILD)/firmware.elf | tail -n2 | grep "^hello world! \\[1, 2, 3, 4, 5, 6, 7, 8, 9, 10\\]eol"

mkboot:
	@echo Make header information for brom booting
	@$(MKSUNXI) $(BUILD)/firmware.bin

include $(TOP)/py/mkrules.mk
