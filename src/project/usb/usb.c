/*
 * usb code for nt124
 *
 * Based on the Black Magic Debug project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>

#include "uart/uart.h"

#include "platform/platform.h"
#include "usb/usb.h"

#define BOARD_IDENT             "NovaTech USB to serial"

#ifdef BOOTLOADER
#define SERIALNO_FLASH_LOCATION ((char*)0x8001ff0)
#else
#define SERIALNO_FLASH_LOCATION "12400000A0\0\0\0\0\0"
#endif

extern struct uart_t uarts[];

usbd_device * usbdev;

static char *get_dev_unique_id(char *serial_no);

static int configured;

static const struct usb_device_descriptor dev = {
        .bLength = USB_DT_DEVICE_SIZE,
        .bDescriptorType = USB_DT_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0xEF,		/* Miscellaneous Device */
        .bDeviceSubClass = 2,		/* Common Class */
        .bDeviceProtocol = 1,		/* Interface Association */
        .bMaxPacketSize0 = 64,
        .idVendor = 0x2aeb,
        .idProduct = 124,
        .bcdDevice = 0x0100,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 3,
        .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor port1_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor port1_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = port1_data_endp,
}};

/* Serial ACM interface */
static const struct usb_endpoint_descriptor port2_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x03,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor port2_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = port2_data_endp,
}};

/* Serial ACM interface */
static const struct usb_endpoint_descriptor port3_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x05,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x85,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor port3_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 2,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = port3_data_endp,
}};

/* Serial ACM interface */
static const struct usb_endpoint_descriptor port4_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x07,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x87,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor port4_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 3,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = port4_data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = port1_data_iface,
}, {
	.num_altsetting = 1,
	.altsetting = port2_data_iface,
}, {
	.num_altsetting = 1,
	.altsetting = port3_data_iface,
}, {
	.num_altsetting = 1,
	.altsetting = port4_data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 4,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

char serial_no[9];

static const char *usb_strings[] = {
	"NovaTech LLC",
	BOARD_IDENT,
	(char *)SERIALNO_FLASH_LOCATION,
	"Port 1",
	"Port 2",
	"Port 3",
	"Port 4",
};

const int uart_lookup[] = {
	ACM0_UART_INDEX,
	ACM1_UART_INDEX,
	ACM2_UART_INDEX,
	ACM3_UART_INDEX
};

static struct uart_t * get_uart_from_index(uint16_t index)
{
	if (index > sizeof(uart_lookup) / sizeof(uart_lookup[0]) - 1)
		return NULL;

	return &uarts[uart_lookup[index]];
}

static enum usbd_request_return_codes nt124_set_control_line_state_callback(usbd_device *dev_,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		usbd_control_complete_callback *complete)
{
	(void)dev_;
	(void)complete;
	(void)buf;
	(void)len;
	struct uart_t *uart;

	if (req->bRequest != USB_CDC_REQ_SET_CONTROL_LINE_STATE)
		return USBD_REQ_NEXT_CALLBACK;

	uart = get_uart_from_index(req->wIndex);
	if (!uart)
		return USBD_REQ_NOTSUPP;

	usbuart_set_control_line_state(uart, req->wValue);

	return USBD_REQ_HANDLED;
}

static enum usbd_request_return_codes nt124_set_line_coding_callback(usbd_device *dev_,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		usbd_control_complete_callback *complete)
{
	(void)dev_;
	(void)complete;
	struct uart_t *uart;

	if (req->bRequest != USB_CDC_REQ_SET_LINE_CODING)
		return USBD_REQ_NEXT_CALLBACK;

	uart = get_uart_from_index(req->wIndex);
	if (!uart)
		return USBD_REQ_NOTSUPP;

	if(*len < sizeof(struct usb_cdc_line_coding))
		return USBD_REQ_NOTSUPP;

	usbuart_set_line_coding(uart, (struct usb_cdc_line_coding*)*buf);

	return USBD_REQ_HANDLED;
}

static enum usbd_request_return_codes nt124_set_flow_control_callback(usbd_device *dev_,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		usbd_control_complete_callback *complete)
{
	(void)dev_;
	(void)complete;
	(void)buf;
	(void)len;
	struct uart_t *uart;

	if (req->bRequest != NT124_USB_REQ_SET_FLOW_CONTROL)
		return USBD_REQ_NEXT_CALLBACK;

	uart = get_uart_from_index(req->wIndex);
	if (!uart)
		return USBD_REQ_NOTSUPP;

	usbuart_set_flow_control(uart, req->wValue);

	return USBD_REQ_HANDLED;
}

uint8_t txempty_buffer;

static enum usbd_request_return_codes nt124_get_txempty_callback(usbd_device *dev_,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		usbd_control_complete_callback *complete)
{
	(void)dev_;
	(void)complete;
	register uint16_t pin_state;
	struct uart_t *uart;

	if (req->bRequest != NT124_USB_REQ_GET_TXEMPTY)
		return USBD_REQ_NEXT_CALLBACK;

	uart = get_uart_from_index(req->wIndex);
	if (!uart)
		return USBD_REQ_NOTSUPP;

	txempty_buffer = usbuart_get_txempty(uart);

	*buf = &txempty_buffer;
	*len = 1;

	return USBD_REQ_HANDLED;
}

static enum usbd_request_return_codes nt124_send_break_callback(usbd_device *dev_,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		usbd_control_complete_callback *complete)
{
	(void)dev_;
	(void)complete;
	(void)buf;
	(void)len;
	struct uart_t *uart;

	if (req->bRequest != NT124_USB_REQ_SEND_BREAK)
		return USBD_REQ_NEXT_CALLBACK;

	uart = get_uart_from_index(req->wIndex);
	if (!uart)
		return USBD_REQ_NOTSUPP;

	if (req->wValue == 0xffff)
		usbuart_send_break(uart);

	return USBD_REQ_HANDLED;
}

int nt124_get_config(void)
{
	return configured;
}

static void nt124_set_config(usbd_device *dev_, uint16_t wValue)
{
	configured = wValue;

	/* Serial interface */
	usbd_ep_setup(dev_, 0x01, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM0_UART_OUT_CALL);
	usbd_ep_setup(dev_, 0x81, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM0_UART_IN_CALL);

	/* Serial interface */
	usbd_ep_setup(dev_, 0x03, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM1_UART_OUT_CALL);
	usbd_ep_setup(dev_, 0x83, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM1_UART_IN_CALL);
	
	/* Serial interface */
	usbd_ep_setup(dev_, 0x05, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM2_UART_OUT_CALL);
	usbd_ep_setup(dev_, 0x85, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM2_UART_IN_CALL);
	
	/* Serial interface */
	usbd_ep_setup(dev_, 0x07, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM3_UART_OUT_CALL);
	usbd_ep_setup(dev_, 0x87, USB_ENDPOINT_ATTR_BULK,
					CDCACM_PACKET_SIZE, ACM3_UART_IN_CALL);

	usbd_register_control_callback(dev_,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_DIRECTION | USB_REQ_TYPE_TYPE,
			nt124_set_control_line_state_callback);

	usbd_register_control_callback(dev_,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_DIRECTION | USB_REQ_TYPE_TYPE,
			nt124_set_line_coding_callback);

	usbd_register_control_callback(dev_,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_DIRECTION | USB_REQ_TYPE_TYPE,
			nt124_set_flow_control_callback);

	usbd_register_control_callback(dev_,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_DIRECTION | USB_REQ_TYPE_TYPE,
			nt124_send_break_callback);

	usbd_register_control_callback(dev_,
			USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_IN,
			USB_REQ_TYPE_DIRECTION | USB_REQ_TYPE_TYPE,
			nt124_get_txempty_callback);
}

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[256];

static char *get_dev_unique_id(char *s)
{
	uint32_t unique_id = *unique_id_p +
			*(unique_id_p + 1) +
			*(unique_id_p + 2);
        int i;

        /* Fetch serial number from chip's unique ID */
        for(i = 0; i < 8; i++) {
                s[7-i] = ((unique_id >> (4*i)) & 0xF) + '0';
        }
        for(i = 0; i < 8; i++)
                if(s[i] > '9')
                        s[i] += 'A' - '9' - 1;
	s[8] = 0;

	return s;
}

void usb_init(void) {
	// reset that the USB is not configured
	configured = 0;
	// reset USB hardware and re-enumerate USB
	usb_platform_init();

	// Initilize the USB stack
	get_dev_unique_id(serial_no);
	usbdev = usbd_init(&USB_DRIVER, &dev, &config, usb_strings,
			    sizeof(usb_strings)/sizeof(char *),
			    usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbdev, nt124_set_config);
	
	// setup the ISR
	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);
}


/*
 * Interrupt routines
 */

void USB_ISR(void) {
	usbd_poll(usbdev);
}
