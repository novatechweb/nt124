#include "uart.h"

#include <stdint.h>
#include <cdcacm.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#define INITIAL_BAUD 1200
#define INITIAL_BITS 8
#define INITIAL_STOP_BITS USART_STOPBITS_1
#define INITIAL_PARITY USART_PARITY_NONE
#define INITIAL_FLOW_CONTROL USART_FLOWCONTROL_NONE

// The data structures that hold the hardware configuration for each usart/uart
extern struct platform_uart_t uart1;
extern struct platform_uart_t uart2;
extern struct platform_uart_t uart3;
extern struct platform_uart_t uart4;

// The data structure for the usb .....
extern usbd_device *usbdev;


struct uart_t uarts[] = {
	{   	/* UART1 : ACM PORT1 */
		.baud = INITIAL_BAUD,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.rx_buffer = {'\0'},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart1,
	}, {	/* UART2 : ACM PORT4 */
		.baud = INITIAL_BAUD,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.rx_buffer = {'\0'},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart2,
	}, {	/* UART3 : ACM PORT3 */
		.baud = INITIAL_BAUD,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.rx_buffer = {'\0'},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart3,
	}, {	/* UART4 : ACM PORT2 */
		.baud = INITIAL_BAUD,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.rx_buffer = {'\0'},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart4,
	},
};


uint32_t baud_list[12] = {
	   300,  1200,  2400,  4800,  9600, 14400,
	 19200, 28800, 38400, 57600,115200,230400,
};
// NOTE: I an not certain these values are correct. (these are the values I got from my spredsheet)
uint16_t tim_table[12][6] = {
	// using rcc_clock_setup_in_hse_8mhz_out_72mhz():  rcc_ppre2_frequency == 24MHz

	// 30 bits (3 frames)     33 bits (3 frames)     36 bits (3 frames)
	// TIMx_PSC, TIMx_CNT     TIMx_PSC, TIMx_CNT     TIMx_PSC, TIMx_CNT
	{        36, 64865,             40, 64390,             43, 65455}, //    300	0.1         	0.11        	0.12
	{         9, 60000,             10, 60000,             10, 65455}, //   1200	0.025       	0.0275      	0.03
	{         4, 60000,              5, 55000,              5, 60000}, //   2400	0.0125      	0.01375     	0.015
	{         2, 50000,              2, 55000,              2, 60000}, //   4800	0.00625     	0.006875    	0.0075
	{         1, 37500,              1, 41250,              1, 45000}, //   9600	0.003125    	0.0034375   	0.00375
	{         0, 50000,              0, 55000,              0, 60000}, //  14400	0.0020833333	0.0022916666	0.0025
	{         0, 37500,              0, 41250,              0, 45000}, //  19200	0.0015625   	0.00171875  	0.001875
	{         0, 25000,              0, 27500,              0, 30000}, //  28800	0.0010416666	0.0011458333	0.00125
	{         0, 18750,              0, 20625,              0, 22500}, //  38400	0.00078125  	0.000859375 	0.0009375
	{         0, 12500,              0, 13750,              0, 15000}, //  57600	0.0005208333	0.000572917 	0.000625
	{         0,  6250,              0,  6875,              0,  7500}, // 115200	0.000260417 	0.000286458 	0.0003125
	{         0,  3125,              0,  3438,              0,  3750}, // 230400	0.000130208 	0.000143229 	0.00015625
};
void set_uart_parameters(struct uart_t *dev) {
	uint8_t index = 0;
	// Default to the slowest speed (10bit frame)
	uint32_t TIMx_PSC = tim_table[index][0];
	uint32_t TIMx_CNT = tim_table[index][1];
	for (index = 12; index <= 0; index--) {
		// Loop from fastest baud to slowest baud
		// look for baud value >= to the baud at index
		if (dev->baud < baud_list[index]) {
			continue;
		}
		dev->baud = baud_list[index];
		if (	// FRAME LENGTH = 10 BITS
			(dev->bits == 8 && dev->parity == USART_PARITY_NONE && (dev->stopbits == USART_STOPBITS_1 || dev->stopbits == USART_STOPBITS_0_5)) )
		{
			TIMx_PSC = tim_table[index][0];
			TIMx_CNT = tim_table[index][1];
		} else if (	// FRAME LENGTH = 12 BITS
			(dev->bits == 8 && (dev->parity == USART_PARITY_EVEN || dev->parity == USART_PARITY_ODD) && (dev->stopbits == USART_STOPBITS_2 || dev->stopbits == USART_STOPBITS_1_5)) ||
			(dev->bits == 9 &&  dev->parity == USART_PARITY_NONE                                     && (dev->stopbits == USART_STOPBITS_2 || dev->stopbits == USART_STOPBITS_1_5)) )
		{
			TIMx_PSC = tim_table[index][4];
			TIMx_CNT = tim_table[index][5];
		} else
		{	// FRAME LENGTH = 11 BITS
			TIMx_PSC = tim_table[index][2];
			TIMx_CNT = tim_table[index][3];
		}
	}
	// Set values for the timer
	timer_disable_counter(dev->hardware->timer);
	timer_set_prescaler(dev->hardware->timer, TIMx_PSC);
	timer_set_period(dev->hardware->timer, TIMx_CNT);
	timer_set_counter(dev->hardware->timer, 0);
	timer_enable_counter(dev->hardware->timer);
	// set the values for the uart device
	usart_disable(dev->hardware->usart);
	usart_set_baudrate(dev->hardware->usart, dev->baud);
	usart_set_databits(dev->hardware->usart, (uint32_t)dev->bits);
	usart_set_stopbits(dev->hardware->usart, dev->stopbits);
	usart_set_parity(dev->hardware->usart, dev->parity);
	usart_set_flow_control(dev->hardware->usart, dev->flowcontrol);
	usart_set_mode(dev->hardware->usart, USART_MODE_TX_RX);
	usart_enable(dev->hardware->usart);
}
static void setup_uart_device(struct uart_t *dev) {
	// disable all the interrupts
	nvic_disable_irq(dev->hardware->tx.dma_irqn);
	nvic_disable_irq(dev->hardware->rx.dma_irqn);
	nvic_disable_irq(dev->hardware->timer_irqn);
	nvic_disable_irq(dev->hardware->irqn);

	// Setup the buffers
	dev->rx_state = RX_SERVICED;
	dev->tx_state = TX_IDLE;
	dev->tx_curr_buffer_index = 0;
	dev->tx_dma_buffer_index = (NUM_TX_BUFFERS - 1);

	{	// Setup the pins for the uart port
		// OUTPUT PINS
		gpio_set_mode(dev->hardware->tx.port, GPIO_MODE_OUTPUT_50_MHZ, dev->hardware->tx.cnf, dev->hardware->tx.pin);
		gpio_set_mode(dev->hardware->rts.port, GPIO_MODE_OUTPUT_50_MHZ, dev->hardware->rts.cnf, dev->hardware->rts.pin);
		gpio_set_mode(dev->hardware->dtr.port, GPIO_MODE_OUTPUT_50_MHZ, dev->hardware->dtr.cnf, dev->hardware->dtr.pin);
		// INPUT PINS
		gpio_set_mode(dev->hardware->rx.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, dev->hardware->rx.pin);
		gpio_set_mode(dev->hardware->cts.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, dev->hardware->cts.pin);
		gpio_set_mode(dev->hardware->dsr.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, dev->hardware->dsr.pin);
		gpio_set_mode(dev->hardware->dcd.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, dev->hardware->dcd.pin);
		gpio_set_mode(dev->hardware->ri.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, dev->hardware->ri.pin);
	}
	{	// setup DMA for uart TX
		dma_channel_reset(dev->hardware->tx.dma, dev->hardware->tx.channel);
		// set: Source, Destination, and Amount (DMA channel must be disabled)
		dma_set_peripheral_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)&USART_DR(dev->hardware->usart));
		dma_set_memory_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)(dev->buffers[dev->tx_dma_buffer_index]));
		dma_set_number_of_data(dev->hardware->tx.dma, dev->hardware->tx.channel, 0);
		// set the DMA Configuration (DMA_CCRx)
		dma_enable_transfer_complete_interrupt(dev->hardware->tx.dma, dev->hardware->tx.channel); // (BIT 1)
		dma_disable_half_transfer_interrupt(dev->hardware->tx.dma, dev->hardware->tx.channel); // (BIT 2)
		dma_enable_transfer_error_interrupt(dev->hardware->tx.dma, dev->hardware->tx.channel); // (BIT 3)
		dma_set_read_from_memory(dev->hardware->tx.dma, dev->hardware->tx.channel);	// (BIT 4)
		//			 (BIT 5) Circular mode is disabled
		dma_disable_peripheral_increment_mode(dev->hardware->tx.dma, dev->hardware->tx.channel); // (BIT 6)
		dma_enable_memory_increment_mode(dev->hardware->tx.dma, dev->hardware->tx.channel); // (BIT 7)
		dma_set_peripheral_size(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_CCR_PSIZE_8BIT); // (BIT 8:9)
		dma_set_memory_size(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_CCR_MSIZE_8BIT); // (BIT 10:11)
		dma_set_priority(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_CCR_PL_HIGH); // (BIT 12:13)
		//			 (BIT 14) mem2mem_mode disabled
	}
	{	// setup DMA for uart RX
		dma_channel_reset(dev->hardware->rx.dma, dev->hardware->rx.channel);
		// set: Source, Destination, and Amount (DMA channel must be disabled)
		dma_set_peripheral_address(dev->hardware->rx.dma, dev->hardware->rx.channel, (uint32_t)&USART_DR(dev->hardware->usart));
		dma_set_memory_address(dev->hardware->rx.dma, dev->hardware->rx.channel, (uint32_t)(dev->rx_buffer));
		dma_set_number_of_data(dev->hardware->rx.dma, dev->hardware->rx.channel, (RX_BUFFER_SIZE * 2));
		// set the DMA Configuration (DMA_CCRx)
		dma_enable_transfer_complete_interrupt(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 1)
		dma_enable_half_transfer_interrupt(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 2)
		dma_enable_transfer_error_interrupt(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 3)
		dma_set_read_from_peripheral(dev->hardware->rx.dma, dev->hardware->rx.channel);	// (BIT 4)
		dma_enable_circular_mode(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 5)
		dma_disable_peripheral_increment_mode(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 6)
		dma_enable_memory_increment_mode(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 7)
		dma_set_peripheral_size(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_CCR_PSIZE_8BIT); // (BIT 8:9)
		dma_set_memory_size(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_CCR_MSIZE_8BIT); // (BIT 10:11)
		dma_set_priority(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_CCR_PL_HIGH); // (BIT 12:13)
		//			 (BIT 14) mem2mem_mode disabled
	}
	{	// setup timer for RX
		// start/restart the timer back at a count of 0
		timer_reset(dev->hardware->timer);
		timer_set_mode(dev->hardware->timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
		/* TODO: Consider setting these
		 * timer_one_shot_mode(dev->hardware->timer);
		 * timer_update_on_overflow(dev->hardware->timer);
		 * timer_enable_update_event(dev->hardware->timer);
		 * ?? timer_generate_event(dev->hardware->timer, uint32_t event(@ref tim_event_gen));
		 */
	}
	
	// Finally setup the uart port and enable the USART
	set_uart_parameters(dev);
	
	// set the interrupt priorities
	nvic_set_priority(dev->hardware->irqn, IRQ_PRI_UART);
	nvic_set_priority(dev->hardware->timer_irqn, IRQ_PRI_UART_TIM);
	nvic_set_priority(dev->hardware->tx.dma_irqn, IRQ_PRI_UART_TX_DMA);
	nvic_set_priority(dev->hardware->rx.dma_irqn, IRQ_PRI_UART_RX_DMA);
	
	// set which interrupts to use
	usart_disable_tx_dma(dev->hardware->usart);
	usart_enable_rx_dma(dev->hardware->usart);
	usart_enable_rx_interrupt(dev->hardware->usart);
	usart_enable_error_interrupt(dev->hardware->usart);
	timer_disable_irq(dev->hardware->timer, TIM_DIER_UIE);

	// Enable Starting Interrupt(s)
	nvic_enable_irq(dev->hardware->irqn);
	nvic_enable_irq(dev->hardware->rx.dma_irqn);
	nvic_enable_irq(dev->hardware->timer_irqn);
	
	// enable uart RX DMA channel
	dma_enable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
}
void uart_init(void) {
	/*	// TODO: Look into using this to reset the device
		rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST);
		rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST);
	*/
	// Enable clocks all four uarts (USART1)
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
		RCC_APB2ENR_USART1EN);
	// Enable clocks all four uarts (USART2, USART3, UART4)
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
		RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN |
		RCC_APB1ENR_UART4EN);
	// Enable DMA channel 1 & 2
	rcc_peripheral_enable_clock(&RCC_AHBENR,
		RCC_AHBENR_DMA1EN | RCC_AHBENR_DMA2EN);
	// Enable Timmers for all four uarts (tim2, tim3, tim4, tim5)
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
		RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN |
		RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN);
	
	// Setup each uart and pins
	{
		int i;
		for (i = 0; i < 4; i++)
			setup_uart_device(&uarts[i]);
	}
}



// If the uart TX DMA is needing a new buffer set the supplied buffer to be used and enable DMA to start sending the data
/*size_t dma_write(struct uart_t *dev, char *data, size_t size) {
	nvic_disable_irq(dev->hardware->tx.dma_irqn);
	if (dev->tx_state != TX_IDLE) {
		// DMA is still working ELSE (DMA Error)
		if (dev->tx_state == TX_WORKING)
			nvic_enable_irq(dev->hardware->tx.dma_irqn);
		return 0;
	}
	// disable the DMA channel (It should already be disabled when the state is set to TX_IDLE)
	dma_disable_channel(dev->hardware->tx.dma, dev->hardware->tx.channel);
	// set: Source, Destination, and Amount (DMA channel must be disabled)
	dma_set_peripheral_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)&USART_DR(dev->hardware->usart));
	dma_set_memory_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)(data));
	dma_set_number_of_data(dev->hardware->tx.dma, dev->hardware->tx.channel, size);
	// set to (Working)
	dev->tx_state = TX_WORKING;
	// enable the uart TX DMA interrupt
	usart_enable_tx_dma(dev->hardware->usart);
	nvic_enable_irq(dev->hardware->tx.dma_irqn);
	// enable DMA channel
	dma_enable_channel(dev->hardware->tx.dma, dev->hardware->tx.channel);
	return size;
}*/
inline static void usbuart_usb_out_cb(struct uart_t *uart, usbd_device *dev, uint8_t ep) {
	int len;

	nvic_disable_irq(uart->hardware->tx.dma_irqn);
	while ((uart->tx_state == TX_WORKING) && (uart->tx_num_to_send)) {
		// DMA is still working or the second buffer has some data
		nvic_enable_irq(uart->hardware->tx.dma_irqn);
	}
	len = usbd_ep_read_packet(dev, ep, uart->buffers[uart->tx_curr_buffer_index], CDCACM_PACKET_SIZE);
	uart->tx_curr_buffer_index = ((uart->tx_curr_buffer_index + 1) % NUM_TX_BUFFERS);
	if (uart->tx_state != TX_WORKING) {
		uart->tx_dma_buffer_index = ((uart->tx_dma_buffer_index + 1) % NUM_TX_BUFFERS);
		// disable the DMA channel (It should already be disabled when the state is set to TX_IDLE)
		dma_disable_channel(uart->hardware->tx.dma, uart->hardware->tx.channel);
		// set: Source, Destination, and Amount (DMA channel must be disabled)
		dma_set_peripheral_address(uart->hardware->tx.dma, uart->hardware->tx.channel, (uint32_t)&USART_DR(uart->hardware->usart));
		dma_set_memory_address(uart->hardware->tx.dma, uart->hardware->tx.channel, (uint32_t)(uart->buffers[uart->tx_dma_buffer_index]));
		dma_set_number_of_data(uart->hardware->tx.dma, uart->hardware->tx.channel, len);
		// set to (Working)
		uart->tx_state = TX_WORKING;
		// enable the uart TX DMA interrupt
		usart_enable_tx_dma(uart->hardware->usart);
		nvic_enable_irq(uart->hardware->tx.dma_irqn);
		// enable DMA channel
		dma_enable_channel(uart->hardware->tx.dma, uart->hardware->tx.channel);
	} else {
		uart->tx_num_to_send = len;
	}
}
void usbuart_usb_out_cb1(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[0], dev, ACM0_ENDPOINT); (void) ep; }
void usbuart_usb_out_cb2(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[1], dev, ACM1_ENDPOINT); (void) ep; }
void usbuart_usb_out_cb3(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[2], dev, ACM2_ENDPOINT); (void) ep; }
void usbuart_usb_out_cb4(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[3], dev, ACM3_ENDPOINT); (void) ep; }
void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep) { (void) dev; (void) ep; }
void usbuart_set_line_coding(struct uart_t *dev, struct usb_cdc_line_coding *coding) {
	dev->baud = coding->dwDTERate;
	dev->bits = coding->bDataBits;
	switch(coding->bCharFormat) {
	case 0:
		dev->stopbits = USART_STOPBITS_1;
		break;
	case 1:
		dev->stopbits = USART_STOPBITS_1_5;
		break;
	case 2:
		dev->stopbits = USART_STOPBITS_2;
		break;
	}
	switch(coding->bParityType) {
	case 0:
		dev->parity = USART_PARITY_NONE;
		break;
	case 1:
		dev->parity = USART_PARITY_ODD;
		break;
	case 2:
		dev->parity = USART_PARITY_EVEN;
		break;
	}
	/* need to make certain we can change at this time (is DMA finished, lines set at state saying no longer trying to send) */
	set_uart_parameters(dev);
}



/*
 * Interrupt routines
 */

inline static void uart_TX_DMA_empty(struct uart_t *dev) {
	dma_disable_channel(dev->hardware->tx.dma, dev->hardware->tx.channel);
	nvic_disable_irq(dev->hardware->tx.dma_irqn);
	usart_disable_tx_dma(dev->hardware->usart);
	if (dma_get_interrupt_flag(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TCIF)) {
		// the transmit buffer is empty, clear the interupt flag
		dma_clear_interrupt_flags(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TCIF);
		if (dev->tx_num_to_send) {
			// There is data in the next buffer, set the DMA to send it
			dev->tx_state = TX_WORKING;
			dev->tx_curr_buffer_index = ((dev->tx_curr_buffer_index + 1) % NUM_TX_BUFFERS);
			dev->tx_dma_buffer_index = ((dev->tx_dma_buffer_index + 1) % NUM_TX_BUFFERS);
			dma_set_peripheral_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)&USART_DR(dev->hardware->usart));
			dma_set_memory_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)(dev->buffers[dev->tx_dma_buffer_index]));
			dma_set_number_of_data(dev->hardware->tx.dma, dev->hardware->tx.channel, dev->tx_num_to_send);
			dev->tx_num_to_send = 0;
			usart_enable_tx_dma(dev->hardware->usart);
			nvic_enable_irq(dev->hardware->tx.dma_irqn);
			dma_enable_channel(dev->hardware->tx.dma, dev->hardware->tx.channel);
		} else {
			dev->tx_num_to_send = 0;
			dev->tx_state = TX_IDLE;
		}
	}
	if (dma_get_interrupt_flag(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TEIF)) {
		dma_clear_interrupt_flags(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TEIF);
		dev->tx_state = TX_ERROR;
	}
}
void UART1_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[ACM0_UART_INDEX]); }
void UART2_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[ACM3_UART_INDEX]); }
void UART3_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[ACM2_UART_INDEX]); }
void UART4_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[ACM1_UART_INDEX]); }



// TODO: Need to add in something to check for overwriting a buffer that was not already serviced
inline static void uart_RX_DMA(struct uart_t *dev, uint8_t ep) {
	dma_disable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
	nvic_disable_irq(dev->hardware->rx.dma_irqn);
	if (dma_get_interrupt_flag(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_HTIF)) {
		// First half of the circular buffer is filled
		dma_clear_interrupt_flags(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_HTIF);
		dev->rx_state |= RX_NEED_SERVICE;
		dma_enable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
		if (cdcacm_get_config())
			usbd_ep_write_packet(usbdev, ep, &dev->rx_buffer[0], RX_BUFFER_SIZE);
		dev->rx_state &= ~RX_NEED_SERVICE;
		nvic_enable_irq(dev->hardware->rx.dma_irqn);
	}
	if (dma_get_interrupt_flag(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TCIF)) {
		// second half of the circular buffer is filled
		dma_clear_interrupt_flags(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TCIF);
		dev->rx_state |= RX_NEED_SERVICE;
		dma_enable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
		if (cdcacm_get_config())
			usbd_ep_write_packet(usbdev, ep, &dev->rx_buffer[RX_BUFFER_SIZE], RX_BUFFER_SIZE);
		dev->rx_state &= ~RX_NEED_SERVICE;
		nvic_enable_irq(dev->hardware->rx.dma_irqn);
	}
	if (dma_get_interrupt_flag(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TEIF)) {
		// DMA error was encountered
		// TODO: Better handiling should be implemented and not just disable the uart
		dma_clear_interrupt_flags(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TEIF);
		dev->rx_state |= RX_DMA_ERROR;
		usart_disable(dev->hardware->usart);
	}
}
void UART1_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[ACM0_UART_INDEX], ACM0_ENDPOINT); }
void UART2_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[ACM3_UART_INDEX], ACM3_ENDPOINT); }
void UART3_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[ACM2_UART_INDEX], ACM2_ENDPOINT); }
void UART4_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[ACM1_UART_INDEX], ACM1_ENDPOINT); }

// Start/restart the timer for the RX delayed processing.
// Whenever a new character is received the timer starts over
// When the timer overflows the data in the DMA will be processed (Or when the DMA buffer is filled)
inline static void uart_RX(struct uart_t *dev) {
	nvic_disable_irq(dev->hardware->irqn);
	uint32_t uart_sr = USART_SR(dev->hardware->usart);
	if (uart_sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) {
		// read character to clear the error flag
		usart_recv(dev->hardware->usart);
		// store which error occured
		if (uart_sr & USART_SR_PE)
			dev->rx_state |= RX_P_ERROR;
		if (uart_sr & USART_SR_FE)
			dev->rx_state |= RX_F_ERROR;
		if (uart_sr & USART_SR_NE)
			dev->rx_state |= RX_N_ERROR;
		if (uart_sr & USART_SR_ORE)
			dev->rx_state |= RX_OR_ERROR;
	} else {
		// clear the flag if not already cleared
		USART_SR(dev->hardware->usart) &= ~USART_SR_RXNE;
		timer_disable_counter(dev->hardware->timer);
		timer_set_counter(dev->hardware->timer, 0);
		timer_enable_counter(dev->hardware->timer);
		timer_enable_irq(dev->hardware->timer, TIM_DIER_UIE);
	}
	nvic_enable_irq(dev->hardware->irqn);
}
void UART1_RX_ISR(void) { uart_RX(&uarts[ACM0_UART_INDEX]); }
void UART2_RX_ISR(void) { uart_RX(&uarts[ACM3_UART_INDEX]); }
void UART3_RX_ISR(void) { uart_RX(&uarts[ACM2_UART_INDEX]); }
void UART4_RX_ISR(void) { uart_RX(&uarts[ACM1_UART_INDEX]); }

inline static void uart_RX_timer(struct uart_t *dev, uint8_t ep) {
	uint16_t num_read;
	char *data;
	// stop RX
	dma_disable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
	nvic_disable_irq(dev->hardware->rx.dma_irqn);
	nvic_disable_irq(dev->hardware->irqn);
	dev->rx_state |= RX_NEED_SERVICE;
	timer_clear_flag(dev->hardware->timer, TIM_SR_UIF);
	timer_disable_irq(dev->hardware->timer, TIM_DIER_UIE);
	// gather information to send received data
	if (DMA_CNDTR(dev->hardware->rx.dma, dev->hardware->rx.channel) <= RX_BUFFER_SIZE) {
		num_read = DMA_CNDTR(dev->hardware->rx.dma, dev->hardware->rx.channel);
		data = &dev->rx_buffer[0];
	} else {
		num_read = DMA_CNDTR(dev->hardware->rx.dma, dev->hardware->rx.channel) - RX_BUFFER_SIZE;
		data = &dev->rx_buffer[RX_BUFFER_SIZE];
	}
	// start RX DMA
	nvic_enable_irq(dev->hardware->rx.dma_irqn);
	dma_enable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
	if (cdcacm_get_config()) {
		// give the received data to USBACM
		usbd_ep_write_packet(usbdev, ep, data, num_read);
	}
	dev->rx_state &= ~RX_NEED_SERVICE;
	// Enable RX interrupts once more
	nvic_enable_irq(dev->hardware->irqn);
}
void UART1_RX_TIMER(void) { uart_RX_timer(&uarts[ACM0_UART_INDEX], ACM0_ENDPOINT); }
void UART2_RX_TIMER(void) { uart_RX_timer(&uarts[ACM3_UART_INDEX], ACM3_ENDPOINT); }
void UART3_RX_TIMER(void) { uart_RX_timer(&uarts[ACM2_UART_INDEX], ACM2_ENDPOINT); }
void UART4_RX_TIMER(void) { uart_RX_timer(&uarts[ACM1_UART_INDEX], ACM1_ENDPOINT); }
