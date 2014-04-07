#include "platform_STM32F1.h"
#include "platform_usb.h"

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>

/* Unused pins */
#define UNUSED_PORTA_PIN1 GPIO6
#define UNUSED_PORTA_PIN2 GPIO7
#define UNUSED_PORTB_PIN1 GPIO7
#define UNUSED_PORTC_PIN1 GPIO0
#define UNUSED_PORTC_PIN2 GPIO1
#define UNUSED_PORTC_PIN3 GPIO4
#define UNUSED_PORTC_PIN4 GPIO13
#define UNUSED_PORTC_PIN5 GPIO15

void platform_init(void) {
	// Setup oscillator
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

#ifdef BOOTLOADER
	// Relocate interrupt vector table here
	SCB_VTOR = 0x2000;
#endif

	// Enable clocks for all GPIO ports. (GPIOA, GPIOB, GPIOC, GPIOD)
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
	    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
	    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN);

	// Set all port pins to input pull-down
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_ALL);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_ALL);
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_ALL);
	gpio_set_mode(GPIOD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO2);
	gpio_clear(GPIOA, GPIO_ALL);
	gpio_clear(GPIOB, GPIO_ALL);
	gpio_clear(GPIOC, GPIO_ALL);
	gpio_clear(GPIOD, GPIO2);

	// 31.4.3 - Internal pull-up and pull-down on JTAG pins
	// ● NJTRST: Input pull-up
	gpio_set_mode(GPIO_BANK_JNTRST, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JNTRST);
	gpio_set(GPIO_BANK_JNTRST, GPIO_JNTRST);
	// ● JTDI: Input pull-up
	gpio_set_mode(GPIO_BANK_JTDI, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JTDI);
	gpio_set(GPIO_BANK_JTDI, GPIO_JTDI);
	// ● JTMS/SWDIO: Input pull-up
	gpio_set_mode(GPIO_BANK_JTMS_SWDIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JTMS_SWDIO);
	gpio_set(GPIO_BANK_JTMS_SWDIO, GPIO_JTMS_SWDIO);
	// ● JTCK/SWCLK: Input pull-down
	gpio_set_mode(GPIO_BANK_JTCK_SWCLK, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JTCK_SWCLK);
	gpio_clear(GPIO_BANK_JTCK_SWCLK, GPIO_JTCK_SWCLK);
	// ● JTDO: Input floating - Already set
	
	{	// DEBUG: LEDs
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, UNUSED_PORTA_PIN1 | UNUSED_PORTA_PIN2);
		gpio_set(GPIOA, UNUSED_PORTA_PIN1 | UNUSED_PORTA_PIN2);
		gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, UNUSED_PORTB_PIN1);
		gpio_set(GPIOB, UNUSED_PORTB_PIN1);
		gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, UNUSED_PORTC_PIN1 | UNUSED_PORTC_PIN2 | UNUSED_PORTC_PIN3);
		gpio_set(GPIOC, UNUSED_PORTC_PIN1 | UNUSED_PORTC_PIN2 | UNUSED_PORTC_PIN3);
	}
}