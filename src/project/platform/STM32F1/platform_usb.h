#ifndef __PLATFORM_USB_H_
#define __PLATFORM_USB_H_

#include <stdint.h>

/* USB */
#define USB_ISR    usb_lp_can_rx0_isr
#define USB_DRIVER stm32f103_usb_driver
#define USB_IRQ    NVIC_USB_LP_CAN_RX0_IRQ

void usb_reset_hardware(void);
void usb_platform_init(void);

#endif /* __PLATFORM_USB_H_ */
