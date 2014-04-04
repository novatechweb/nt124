#include "platform_usb.h"

#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* DFU/Boot1 */
#define DFU_PORT GPIOB
#define DFU_PIN GPIO2
/* USB */
#define USB_DATA_PORT  GPIOA
#define USB_DATA_P_PIN GPIO12
#define USB_DATA_N_PIN GPIO11
#define USB_DRIVER stm32f103_usb_driver
#define USB_IRQ    NVIC_USB_LP_CAN_RX0_IRQ
/* re-enumerate pin */
#define USB_ENUMERATE_PORT GPIOA
#define USB_ENUMERATE_PIN  GPIO8

volatile uint32_t *unique_id_p = (volatile uint32_t *)0x1FFFF7E8;

void assert_boot_pin(void) {
	// Set the DFU pin
	gpio_set_mode(DFU_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, DFU_PIN);
	gpio_clear(DFU_PORT, DFU_PIN);
}

void usb_platform_init(void)
{
	// Note: http://www.usbmadesimple.co.uk/ums_3.htm
	
	// TODO: Make certain the USB hardware is disabled
	/*	// TODO: Look into using this to reset the device
		rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST);
		rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST);
	*/

	// USB to detached - pull-down both data lines
	gpio_set_mode(USB_DATA_PORT, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_ENUMERATE_PIN | USB_DATA_P_PIN | USB_DATA_N_PIN);
	gpio_clear(USB_DATA_PORT, USB_ENUMERATE_PIN | USB_DATA_P_PIN | USB_DATA_N_PIN);
	// the re-enumerate is just floating for now
	gpio_set_mode(USB_ENUMERATE_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_ENUMERATE_PIN);

	// wait a short bit for detatch to be detected
	{
		uint16_t i;
		for (i = 0; i < 10000; i++)
			__asm__("nop");
	}

	// release the USB data lines (set back to FLOAT/PULL_UPDOWN inputs)
	gpio_set_mode(USB_DATA_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_DATA_P_PIN | USB_DATA_N_PIN);
	// set into Idle state (pull-up resistor on the re-enumerate pin)
	gpio_set_mode(USB_ENUMERATE_PORT, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_ENUMERATE_PIN);
	gpio_set(USB_ENUMERATE_PORT, USB_ENUMERATE_PIN);

}
