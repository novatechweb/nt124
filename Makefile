PREFIX ?= arm-cortexm3-eabi
CROSS_COMPILE ?= $(PREFIX)-
PLATFORM ?= STM32F1

all:	libopencm3
	@$(MAKE) CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM) -C src

gdb: all
	@$(MAKE) CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM) -C src gdb

git_init:
	@git init
	@git add ./*
	@git commit -sm 'commiting initial files for $(PROGRAM)'

submodule: git_init
	@git submodule add https://github.com/libopencm3/libopencm3.git ./libopencm3
	@git submodule update --init
	@git commit ./libopencm3 -m "Added library as a submodule"
	@rm -rf ./libopencm3

libopencm3:
	@git submodule init
	@git submodule update
	@$(MAKE) PREFIX=$(PREFIX) -C libopencm3 lib

clean:
	@$(MAKE) CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM) -C src clean

distclean: clean
	-@rm -rf libopencm3

.PHONY:	all clean distclean git_init submodule gdb
