#ifndef __CDCACM_H_
#define __CDCACM_H_

#include <stdint.h>

#include <platform.h>
#include <uart.h>

#define ACM0_UART_INDEX 0
#define ACM0_UART_OUT_CALL usbuart_usb_out_cb1
#define ACM0_ENDPOINT 1

#define ACM1_UART_INDEX 3
#define ACM1_UART_OUT_CALL usbuart_usb_out_cb4
#define ACM1_ENDPOINT 3

#define ACM2_UART_INDEX 2
#define ACM2_UART_OUT_CALL usbuart_usb_out_cb3
#define ACM2_ENDPOINT 5

#define ACM3_UART_INDEX 1
#define ACM3_UART_OUT_CALL usbuart_usb_out_cb2
#define ACM3_ENDPOINT 7

extern volatile uint32_t *unique_id_p;

void cdcacm_init(void);
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

#endif /* __CDCACM_H_ */
