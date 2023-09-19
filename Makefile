# -*-makefile-*-
# $Id: script.make $
#

#BOOTLOADER := YES

include VERSION.mk

PROJECT = nt124
DEVICE = stm32f103rct6
SRC_DIR = src
OPENCM3_DIR = $(SRC_DIR)/libopencm3
BUILD_DIR = build

OPT = -O3
CSTD = -std=c11
CFLAGS += -g
CFLAGS += -Wextra
CFLAGS += -Werror
CFLAGS += -Wstrict-prototypes
CFLAGS += -Wmissing-prototypes
CFLAGS += -DVERSION_MAJOR=$(VERSION_MAJOR)
CFLAGS += -DVERSION_MINOR=$(VERSION_MINOR)


# replace `all` with `all_primary` as the primary target because we need other targets then just building the image
#   need to make certain the git submodules are checked out and built
all_primary: $(OPENCM3_DIR)/lib/libopencm3_stm32f1.a
	@$(MAKE) all

include $(SRC_DIR)/project/include.mk

-include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(SRC_DIR)/rules.mk
-include $(OPENCM3_DIR)/mk/genlink-rules.mk

libopencm3: $(OPENCM3_DIR)/lib/libopencm3_stm32f1.a

# Make certain libopencm3 is built for stm32f1 series chips
$(OPENCM3_DIR)/lib/libopencm3_stm32f1.a: $(OPENCM3_DIR)/.git
	@$(MAKE) --directory=$(OPENCM3_DIR) TARGETS=stm32/f1

# make certain libopencm3 is checked out
$(OPENCM3_DIR)/.git:
	@git submodule update --init --recursive

flash: $(PROJECT).flash

%.flash: %.elf
	@printf "  FLASH\t$<\n"
	$(Q)$(PREFIX)gdb --command=scripts/gdb_flash_cmds $^

#%.flash: %.bin
#	@printf "  FLASH\t$<\n"
#	$(Q)st-flash $(FLASHSIZE) write $^ 0x8000000

gdb: $(PROJECT).elf
	$(Q)cgdb -d $(PREFIX)gdb -- --command=scripts/gdb_cmds $^

dfu: $(PROJECT).dfu

%.dfu: %.bin
	@printf "  DFU Load\t$<\n"
	$(Q)dfu-util --device 2aeb:0000 --dfuse-address 0x08002000:leave --download $^

flashDFU: stm32f1_dfu.serial
	@printf "  FLASH\t$<\n"
	$(Q)$(PREFIX)gdb --command=$(SRC_DIR)/../scripts/gdb_flash_cmds $^

format:
	$(Q)clang-format --style=WebKit -i $(SRC_DIR)/project/{,*,*/*,*/*/*}/*{.c,.h}

print-%:
	@echo $*=$($*)

distclean: clean
	@$(MAKE) --directory=$(OPENCM3_DIR) clean

.PHONY: distclean libopencm3 flash
# vim: syntax=make
