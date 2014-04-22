PROGRAM := usbacm
BUILDDATE := `date +"%Y%m%d"`
VERSION := 0.1
VERSION_SUFFIX := NONE

CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += \
	-O0 \
	-g3 \
	-MD \
	-std=gnu99 \
	-Wall \
	-Wextra \
	-Wno-pointer-sign \
	-Wno-char-subscripts\
	-Wno-sign-compare \
	-DBUILDDATE=\"$(BUILDDATE)\" \
	-DVERSION=\"$(VERSION)\" \
	-DVERSION_SUFFIX=\"$(VERSION_SUFFIX)\" \
	-I../libopencm3/include \
	-I.

LDFLAGS += \
	-lc \
	-lnosys \
	-nostartfiles \
	-Wl,-Map=mapfile \
	-Wl,-gc-sections \
	-l$(LIB_OPENCM32_OBJECT) \
	-L../libopencm3/lib \

# uncomment the next two lines to use the USB_DFU bootloader
CFLAGS += -DBOOTLOADER
LDFLAGS += -Wl,-Ttext=0x8002000 -Wl,--defsym,_stack=0x20005000

Q := @
# Program rules
SUB_DIR = \
	platform/$(PLATFORM) \
	cdcacm \
	uart

include $(addsuffix /Makefile, $(SUB_DIR))

SRC += \
	main.c

OBJ = $(SRC:.c=.o)

all: $(PROGRAM).bin $(PROGRAM).hex

gdb: $(PROGRAM).elf
	cgdb -d arm-cortexm3-eabi-gdb $^

upload: all
	scp usbacm.bin root@172.16.200.3:/lib/firmware/ntserial.bin

$(PROGRAM).bin: $(PROGRAM).elf
	@printf '  %s  -->  %s\n' $^ $@
	$(Q)$(OBJCOPY) -O binary $^ $@

$(PROGRAM).hex: $(PROGRAM).elf
	@printf '  %s  -->  %s\n' $^ $@
	$(Q)$(OBJCOPY) -O ihex $^ $@

$(PROGRAM).elf: $(OBJ)
	@printf '\n'
	@printf '  %s  -->  %s\n' "$^" "$@"
	$(Q)$(CC) -o $@ $^ $(LDFLAGS)

clean:
	-$(Q)$(RM) $(addsuffix /*.o, $(SUB_DIR) .)
	-$(Q)$(RM) $(addsuffix /*.d, $(SUB_DIR) .)
	-$(Q)$(RM) $(addsuffix /*.~, $(SUB_DIR) .)
	-$(Q)$(RM) $(addsuffix /mapfile, $(SUB_DIR) .)
	-$(Q)$(RM) $(PROGRAM).elf $(PROGRAM).bin $(PROGRAM).hex

.PHONY:	all clean gdb upload

-include *.d
