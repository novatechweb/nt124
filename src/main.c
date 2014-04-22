#include <stdlib.h>
#include <assert.h>
//#include <stdint.h>
//#include <stdbool.h>

#include "platform/platform.h"
#include "cdcacm/cdcacm.h"
#include "uart/uart.h"

int
main(int argc, char **argv) {
	(void) argc;
	(void) argv;

	platform_reset_hardware();
	platform_init();
	uart_init();
	cdcacm_init();

	while(1) {
		__asm__("nop");
		// wait for a second
		platform_delay(10);
		test();
	}
	
	/* Should never get here */
	return EXIT_SUCCESS;
}
