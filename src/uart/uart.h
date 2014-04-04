#ifndef __UART_H_
#define __UART_H_

#include <stdint.h>
#include <string.h>

//#include <cdcacm.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <platform.h>

typedef enum {
	RX_SERVICED,                // RX buffer has been taken care
	RX_NEED_SERVICE,            // RX buffer needs serviced
	RX_BUFF_ERROR = (1 << 1),   // RX Buffer error (uart stopped)
	RX_DMA_ERROR = (1 << 2),    // RX DMA error (uart stopped)
	RX_P_ERROR = (1 << 3), 	    // RX Parity Error
	RX_F_ERROR = (1 << 4), 	    // RX Framing Error
	RX_N_ERROR = (1 << 5), 	    // RX Noise Error
	RX_OR_ERROR = (1 << 6),     // RX Overrun Error (data was lost)
	RX_TYPE_COUNT,
	RX_TYPE_UNKNOWN = RX_TYPE_COUNT
} rx_state_type_e;
typedef enum {
	TX_IDLE,        // Idle, tx_dma_buffer_index data has been transmitted
	TX_WORKING,     // Working, tx_dma_buffer_index belongs to DMA
	TX_ERROR,       // DMA Error, Transmission stoped!
	TX_TYPE_COUNT,
	TX_TYPE_UNKNOWN = TX_TYPE_COUNT
} tx_state_type_e;

// the format of the usarts/uarts buffer data structure
struct uart_t {
	uint32_t baud;
	uint32_t stopbits;
	uint32_t parity;
	uint32_t flowcontrol;
	uint8_t bits;
	/* RX variables */
	volatile rx_state_type_e rx_state;
	// The asumption is made that we can handle one of the buffers before the other buffer is filled
	char rx_buffer[(RX_BUFFER_SIZE * 2)];
	/* TX variables */
	volatile tx_state_type_e tx_state;
	uint16_t tx_num_to_send;
	uint8_t tx_curr_buffer_index;	// buffers index to place new characters into.
	uint8_t tx_dma_buffer_index;	// buffers index currently being used by DMA.
	char buffers[NUM_TX_BUFFERS][TX_BUFFER_SIZE];
	/* Hardware register values for UART */
	struct platform_uart_t *hardware;
} __attribute__((packed));

void uart_init(void);
void usbuart_set_line_coding(struct uart_t *dev, struct usb_cdc_line_coding *coding);
void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep);
void usbuart_usb_out_cb1(usbd_device *dev, uint8_t ep);
void usbuart_usb_out_cb2(usbd_device *dev, uint8_t ep);
void usbuart_usb_out_cb3(usbd_device *dev, uint8_t ep);
void usbuart_usb_out_cb4(usbd_device *dev, uint8_t ep);

#endif /* __UART_H_ */
