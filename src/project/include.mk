CFILES += main.c
CFILES += platform/STM32F1/platform_STM32F1.c
CFILES += platform/STM32F1/platform_uart.c
CFILES += platform/STM32F1/platform_usb.c
CFILES += uart/uart.c
CFILES += usb/usb.c

VPATH += $(SRC_DIR)/project/
INCLUDES += -I$(SRC_DIR)/project/

ifeq ($(BOOTLOADER),YES)
CFLAGS += -DBOOTLOADER
LDFLAGS += -Wl,-Ttext=0x8002000 -Wl,--defsym,_stack=0x20005000
endif
