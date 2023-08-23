#include <stdlib.h>
#include <assert.h>

#include "platform/platform.h"
#include "usb/usb.h"
#include "uart/uart.h"

int
main(int argc, char **argv) {
	(void) argc;
	(void) argv;

	platform_reset_hardware();
	platform_init();
	uart_init();
	usb_init();

	while(1) {
		__asm__("nop");
	}
	
	/* Should never get here */
	return EXIT_SUCCESS;
}
