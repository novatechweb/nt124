MCPU = cortex-m3

LOWER_PLATFORM = `tr "[:upper:]" "[:lower:]" <<< "$(PLATFORM)"`
LIB_OPENCM32_OBJECT = opencm3_$(LOWER_PLATFORM)

CFLAGS += \
	-mcpu=$(MCPU) \
	-mthumb \
	-D$(PLATFORM) \
	-DINLINE_GPIO \
	-Iplatform \
	-Iplatform/$(PLATFORM)

LDFLAGS += \
	-mthumb \
	-mcpu=$(MCPU) \
	-Wl,-T,platform/$(PLATFORM)/program_$(PLATFORM).ld

SRC += \
	platform/STM32F1/platform_STM32F1.c \
	platform/STM32F1/platform_uart.c \
	platform/STM32F1/platform_usb.c 

VPATH += platform/STM32F1/
