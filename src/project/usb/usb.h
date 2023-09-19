#ifndef __NT124_H_
#define __NT124_H_

#include <stdint.h>

#include "platform/platform.h"
#include "uart/uart.h"

#define ACM0_UART_INDEX 0
#define ACM0_UART_OUT_CALL usbuart_usb_out_cb1
#define ACM0_UART_IN_CALL usbuart_usb_in_cb1
#define ACM0_ENDPOINT 1

#define ACM1_UART_INDEX 3
#define ACM1_UART_OUT_CALL usbuart_usb_out_cb4
#define ACM1_UART_IN_CALL usbuart_usb_in_cb4
#define ACM1_ENDPOINT 3

#define ACM2_UART_INDEX 2
#define ACM2_UART_OUT_CALL usbuart_usb_out_cb3
#define ACM2_UART_IN_CALL usbuart_usb_in_cb3
#define ACM2_ENDPOINT 5

#define ACM3_UART_INDEX 1
#define ACM3_UART_OUT_CALL usbuart_usb_out_cb2
#define ACM3_UART_IN_CALL usbuart_usb_in_cb2
#define ACM3_ENDPOINT 7

#define NT124_USB_REQ_SEND_BREAK	0x23
#define NT124_USB_REQ_SET_FLOW_CONTROL	0x90
#define NT124_USB_REQ_GET_TXEMPTY	0x91

extern volatile uint32_t *unique_id_p;

void usb_init(void);
int nt124_get_config(void);
int nt124_get_dtr(void);
void usb_task(void);

#endif /* __NT124_H_ */
