#include "uart.h"

#include <stdint.h>
#include <usb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/usb.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/systick.h>

#define INITIAL_BAUD 1200
#define INITIAL_BITS 8
#define INITIAL_STOP_BITS USART_STOPBITS_1
#define INITIAL_PARITY USART_PARITY_NONE
#define INITIAL_FLOW_CONTROL USART_FLOWCONTROL_NONE


// The data structure for the usb .....
extern usbd_device *usbdev;

// The data structures that hold the hardware configuration for each usart/uart
extern struct platform_uart_t uart1;
extern struct platform_uart_t uart2;
extern struct platform_uart_t uart3;
extern struct platform_uart_t uart4;

volatile bool g_systick_in_use = false;

struct uart_t uarts[] = {
	{   	/* UART1 : ACM PORT1 */
		.baud = INITIAL_BAUD,
		.baud_index = 0,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.num_read = 0,
		.rx_dma_index = 0,
		.rx_buffer = {{'\0'},{'\0'},},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart1,
		.ep = ACM0_ENDPOINT,
		.usb_in_tx_state = USB_TX_IDLE,
		.tx_empty_count_down = -1,
		.tx_empty = true,
		.ctrl_count_down = -1,
		.ctrl_update_retries = 0,
#ifdef UART_TRACE_ENABLED
		.trace_count = 0,
#endif
	}, {	/* UART2 : ACM PORT4 */
		.baud = INITIAL_BAUD,
		.baud_index = 0,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.num_read = 0,
		.rx_dma_index = 0,
		.rx_buffer = {{'\0'},{'\0'},},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart2,
		.ep = ACM3_ENDPOINT,
		.usb_in_tx_state = USB_TX_IDLE,
		.tx_empty_count_down = -1,
		.tx_empty = true,
		.ctrl_count_down = -1,
		.ctrl_update_retries = 0,
#ifdef UART_TRACE_ENABLED
		.trace_count = 0,
#endif
	}, {	/* UART3 : ACM PORT3 */
		.baud = INITIAL_BAUD,
		.baud_index = 0,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.num_read = 0,
		.rx_dma_index = 0,
		.rx_buffer = {{'\0'},{'\0'},},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart3,
		.ep = ACM2_ENDPOINT,
		.usb_in_tx_state = USB_TX_IDLE,
		.tx_empty_count_down = -1,
		.tx_empty = true,
		.ctrl_count_down = -1,
		.ctrl_update_retries = 0,
#ifdef UART_TRACE_ENABLED
		.trace_count = 0,
#endif
	}, {	/* UART4 : ACM PORT2 */
		.baud = INITIAL_BAUD,
		.baud_index = 0,
		.bits = INITIAL_BITS,
		.stopbits = INITIAL_STOP_BITS,
		.parity = INITIAL_PARITY,
		.flowcontrol = INITIAL_FLOW_CONTROL,
		/* RX variables */
		.rx_state = RX_SERVICED,
		.num_read = 0,
		.rx_dma_index = 0,
		.rx_buffer = {{'\0'},{'\0'},},
		/* TX variables */
		.tx_state = TX_IDLE,
		.buffers = {{'\0'},{'\0'},},
		.tx_curr_buffer_index = 0,
		.tx_dma_buffer_index = (NUM_TX_BUFFERS - 1),
		/* Hardware register values for UART */
		.hardware = &uart4,
		.ep = ACM1_ENDPOINT,
		.usb_in_tx_state = USB_TX_IDLE,
		.tx_empty_count_down = -1,
		.tx_empty = true,
		.ctrl_count_down = -1,
		.ctrl_update_retries = 0,
#ifdef UART_TRACE_ENABLED
		.trace_count = 0,
#endif
	},
};


uint32_t baud_list[] = {
	   300,  1200,  2400,  4800,  9600, 14400,
	 19200, 28800, 38400, 57600,115200,230400,
};
// NOTE: I an not certain these values are correct. (these are the values I got from my spredsheet)
uint32_t tim_table[][6] = {
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

/* Time to delay TX empty notification, rounded up to nearest millisecond */
int tx_empty_delay[] = {
	32, //    300
	8, //   1200
	4, //   2400
	2, //   4800
	1, //   9600
	1, //  14400
	1, //  19200
	1, //  28800
	1, //  38400
	1, //  57600
	1, // 115200
	1, // 230400
};

/* Time to buffer ctrl line changes rounded up to nearest millisecond */
int ctrl_delay[] = {
	512, //    300
	256, //   1200
	128, //   2400
	64, //   4800
	32, //   9600
	22, //  14400
	16, //  19200
	11, //  28800
	8, //  38400
	6, //  57600
	3, // 115200
	2, // 230400
};

uint16_t uart_get_control_line_state(struct uart_t *dev);
void send_rx(struct uart_t *dev);

/*
 * uart trace
 */
#ifdef UART_TRACE_ENABLED
void do_uart_trace(struct uart_t *dev, int line) {
	cm_disable_interrupts();
	dev->trace_buf[dev->trace_count % UART_TRACE_BUF_SIZE] = (uint16_t)line;
	dev->trace_count++;
	cm_enable_interrupts();
}
#endif

/*
 * systick routines
 */

void schedule_ctrl_update(struct uart_t *dev, bool tx_empty) {
	//FIXME
	if (nt124_get_config()) {
		cm_disable_interrupts();

		/* Set appropriate down counter if it isn't already counting */
		if (tx_empty && dev->tx_empty_count_down == -1)
			dev->tx_empty_count_down = tx_empty_delay[dev->baud_index];
		else if(!tx_empty && dev->ctrl_count_down == -1)
			dev->ctrl_count_down = ctrl_delay[dev->baud_index];

		if (!g_systick_in_use) {
			systick_interrupt_enable();
			systick_counter_enable();
			g_systick_in_use = true;
		}

		cm_enable_interrupts();
		UART_TRACE(dev, __LINE__);
	}
}

void disable_systick_if_unused() {
	int i;
	bool systick_in_use = false;
	
	for (i = 0; i < sizeof(uarts)/sizeof(uarts[0]); i++) {
		if (uarts[i].tx_empty_count_down != -1 || uarts[i].ctrl_count_down != -1) {
			systick_in_use = true;
			break;
		}
	}

	if (!systick_in_use) {
		systick_interrupt_disable();
		systick_counter_disable();
		g_systick_in_use = false;
	}
}

void raw_ctrl_update_sent(struct uart_t *dev, bool tx_empty) {
	if (tx_empty)
		dev->tx_empty_count_down = -1;
	
	dev->ctrl_count_down = -1;
}

void ctrl_update_sent(struct uart_t *dev, bool tx_empty) {
	cm_disable_interrupts();

	if (g_systick_in_use) {
		raw_ctrl_update_sent(dev, tx_empty);

		disable_systick_if_unused();
	}

	cm_enable_interrupts();
	UART_TRACE(dev, __LINE__);
}

bool send_ctrl_update(struct uart_t *dev, bool tx_empty) {
	uint16_t reply;

	if (dev->usb_in_tx_state == USB_TX_IDLE) {
		UART_TRACE(dev, __LINE__);
		reply = uart_get_control_line_state(dev);
		if (dev->tx_empty)
			reply |= ACM_CTRL_TXEMPTY;

		if (usbd_ep_write_packet(usbdev, dev->ep, &reply, 2) == 0) {
			dev->ctrl_update_retries++;
			if( dev->ctrl_update_retries < 50 )
				return false;
		}
		dev->ctrl_update_retries = 0;

		raw_ctrl_update_sent(dev, tx_empty);
		UART_TRACE(dev, __LINE__);
		return true;
	}
	
	return false;
}

void sys_tick_handler(void) {
	int i;
	bool update_sent = false;
	
	for (i = 0; i < sizeof(uarts)/sizeof(uarts[0]); i++) {
		if (uarts[i].tx_empty_count_down == 0) {
			if (uarts[i].tx_state == TX_IDLE)
				uarts[i].tx_empty = true;
			if (send_ctrl_update(&uarts[i], true))
				update_sent = true;
		} else {
			if (uarts[i].tx_empty_count_down > 0) {
				uarts[i].tx_empty_count_down--;
			}

			if (uarts[i].ctrl_count_down == 0) {
				if (send_ctrl_update(&uarts[i], false))
					update_sent = true;
			} else if (uarts[i].ctrl_count_down > 0) {
				uarts[i].ctrl_count_down--;
			}
		}
	}

	if (update_sent)
		disable_systick_if_unused();
}

/*
 * uart routines
 */

void set_uart_parameters(struct uart_t *dev) {
	int index = 0;
	// Default to the slowest speed (10bit frame)
	uint32_t TIMx_PSC = tim_table[index][0];
	uint32_t TIMx_CNT = tim_table[index][1];
	UART_TRACE(dev, __LINE__);
	for (index = sizeof(tim_table)/sizeof(tim_table[0]) - 1; index >= 0; index--) {
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
		break;
	}
	dev->baud_index = index;
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

	schedule_ctrl_update(dev, false);
}
static void disable_uart_irqs(struct uart_t *dev) {
	// disable all the interrupts
	nvic_disable_irq(dev->hardware->tx.dma_irqn);
	nvic_disable_irq(dev->hardware->rx.dma_irqn);
	nvic_disable_irq(dev->hardware->irqn);
	nvic_disable_irq(dev->hardware->timer_irqn);
	nvic_disable_irq(dev->hardware->cts.irqn);
	nvic_disable_irq(dev->hardware->dsr.irqn);
	nvic_disable_irq(dev->hardware->dcd.irqn);
	if (dev->hardware->ri.irqn != NVIC_IRQ_COUNT)
		nvic_disable_irq(dev->hardware->ri.irqn);
}
static void setup_uart_starting_interrupts(struct uart_t *dev) {
	// set which interrupts to use
	usart_disable_tx_dma(dev->hardware->usart);
	usart_enable_rx_dma(dev->hardware->usart);
	usart_enable_rx_interrupt(dev->hardware->usart);
	usart_enable_error_interrupt(dev->hardware->usart);
	timer_disable_irq(dev->hardware->timer, TIM_DIER_UIE);

	// Enable Starting Interrupt(s)
	nvic_disable_irq(dev->hardware->tx.dma_irqn);
	nvic_enable_irq(dev->hardware->rx.dma_irqn);
	nvic_enable_irq(dev->hardware->irqn);
	nvic_enable_irq(dev->hardware->timer_irqn);
	nvic_enable_irq(dev->hardware->cts.irqn);
	nvic_enable_irq(dev->hardware->dsr.irqn);
	nvic_enable_irq(dev->hardware->dcd.irqn);
	if (dev->hardware->ri.irqn != NVIC_IRQ_COUNT)
		nvic_enable_irq(dev->hardware->ri.irqn);
}
static void setup_uart_device(struct uart_t *dev) {
	// Setup the buffers
	dev->rx_state = RX_SERVICED;
	dev->tx_state = TX_IDLE;
	dev->tx_curr_buffer_index = 0;
	dev->tx_dma_buffer_index = (NUM_TX_BUFFERS - 1);

	{	// Setup the pins for the uart port
		// OUTPUT PINS
		gpio_set_mode(dev->hardware->tx.port, GPIO_MODE_OUTPUT_50_MHZ, dev->hardware->tx.cnf, dev->hardware->tx.pin);
		gpio_set_mode(dev->hardware->rts.port, GPIO_MODE_OUTPUT_50_MHZ, dev->hardware->rts.cnf, dev->hardware->rts.pin);
		gpio_set_mode(dev->hardware->dir.port, GPIO_MODE_OUTPUT_50_MHZ, dev->hardware->dir.cnf, dev->hardware->dir.pin);
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
		dma_set_memory_address(dev->hardware->rx.dma, dev->hardware->rx.channel, (uint32_t)(dev->rx_buffer[0]));
		dma_set_number_of_data(dev->hardware->rx.dma, dev->hardware->rx.channel, RX_BUFFER_SIZE);
		// set the DMA Configuration (DMA_CCRx)
		dma_enable_transfer_complete_interrupt(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 1)
		//dma_enable_half_transfer_interrupt(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 2)
		dma_enable_transfer_error_interrupt(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 3)
		dma_set_read_from_peripheral(dev->hardware->rx.dma, dev->hardware->rx.channel);	// (BIT 4)
		//dma_enable_circular_mode(dev->hardware->rx.dma, dev->hardware->rx.channel); // (BIT 5)
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
	{	// setup external interrupts for CTS, DSR, DCD and RI
		// CTS
		exti_select_source(dev->hardware->cts.pin, dev->hardware->cts.port);
		exti_set_trigger(dev->hardware->cts.pin, EXTI_TRIGGER_BOTH);
		exti_enable_request(dev->hardware->cts.pin);
		// DSR
		exti_select_source(dev->hardware->dsr.pin, dev->hardware->dsr.port);
		exti_set_trigger(dev->hardware->dsr.pin, EXTI_TRIGGER_BOTH);
		exti_enable_request(dev->hardware->dsr.pin);
		// DCD
		exti_select_source(dev->hardware->dcd.pin, dev->hardware->dcd.port);
		exti_set_trigger(dev->hardware->dcd.pin, EXTI_TRIGGER_BOTH);
		exti_enable_request(dev->hardware->dcd.pin);
		if (dev->hardware->ri.irqn != NVIC_IRQ_COUNT) {
			// setup external interrupts for RI if there is one
			exti_select_source(dev->hardware->ri.pin, dev->hardware->ri.port);
			exti_set_trigger(dev->hardware->ri.pin, EXTI_TRIGGER_BOTH);
			exti_enable_request(dev->hardware->ri.pin);
		}
	}
	
	// Finally setup the uart port and enable the USART
	set_uart_parameters(dev);
	
	// set the interrupt priorities
	nvic_set_priority(dev->hardware->irqn, IRQ_PRI_UART);
	nvic_set_priority(dev->hardware->timer_irqn, IRQ_PRI_UART_TIM);
	nvic_set_priority(dev->hardware->tx.dma_irqn, IRQ_PRI_UART_TX_DMA);
	nvic_set_priority(dev->hardware->rx.dma_irqn, IRQ_PRI_UART_RX_DMA);
	nvic_set_priority(dev->hardware->cts.irqn, IRQ_PRI_EXT_INT);
	nvic_set_priority(dev->hardware->dsr.irqn, IRQ_PRI_EXT_INT);
	nvic_set_priority(dev->hardware->dcd.irqn, IRQ_PRI_EXT_INT);
	if (dev->hardware->ri.irqn != NVIC_IRQ_COUNT)
		nvic_set_priority(dev->hardware->ri.irqn, IRQ_PRI_EXT_INT);
	
	// enable uart RX DMA channel
	dma_enable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
}
void uart_init(void) {
	int i;
	// disable irqs
	for (i = 0; i < sizeof(uarts)/sizeof(uarts[0]); i++)
		disable_uart_irqs(&uarts[i]);
	uart_platform_init();
	// Setup each uart and pins
	for (i = 0; i < sizeof(uarts)/sizeof(uarts[0]); i++)
		setup_uart_device(&uarts[i]);
	// setup interrupts
	for (i = 0; i < sizeof(uarts)/sizeof(uarts[0]); i++)
		setup_uart_starting_interrupts(&uarts[i]);
}



inline static void usbuart_usb_out_cb(struct uart_t *uart, usbd_device *dev) {
	int len;

	nvic_disable_irq(uart->hardware->tx.dma_irqn);
	UART_TRACE(uart, __LINE__);
	if ((uart->tx_state == TX_WORKING || uart->tx_state == TX_FULL) && (uart->tx_num_to_send)) {
		// DMA is still working or the second buffer has some data
		uart->tx_state = TX_FULL;
		/* Clear the CTR_RX bit so we aren't notified again */
		USB_CLR_EP_RX_CTR(uart->ep);
		UART_TRACE(uart, __LINE__);
	} else {
		UART_TRACE(uart, __LINE__);
		len = usbd_ep_read_packet(dev, uart->ep, uart->buffers[uart->tx_curr_buffer_index], CDCACM_PACKET_SIZE);
		if ((uart->tx_state != TX_WORKING) && (len)) {
			uart->tx_empty = false;
			uart->tx_dma_buffer_index = uart->tx_curr_buffer_index;
			// disable the DMA channel (It should already be disabled when the state is set to TX_IDLE)
			dma_disable_channel(uart->hardware->tx.dma, uart->hardware->tx.channel);
			// set: Source, Destination, and Amount (DMA channel must be disabled)
			dma_set_peripheral_address(uart->hardware->tx.dma, uart->hardware->tx.channel, (uint32_t)&USART_DR(uart->hardware->usart));
			dma_set_memory_address(uart->hardware->tx.dma, uart->hardware->tx.channel, (uint32_t)(uart->buffers[uart->tx_dma_buffer_index]));
			dma_set_number_of_data(uart->hardware->tx.dma, uart->hardware->tx.channel, len);
			uart->tx_curr_buffer_index = ((uart->tx_curr_buffer_index + 1) % NUM_TX_BUFFERS);
			// set to (Working)
			uart->tx_state = TX_WORKING;
			// enable the uart TX DMA interrupt
			usart_enable_tx_dma(uart->hardware->usart);
			// enable DMA channel
			dma_enable_channel(uart->hardware->tx.dma, uart->hardware->tx.channel);
			UART_TRACE(uart, __LINE__);
		} else if (len) {
			uart->tx_num_to_send = len;
			UART_TRACE(uart, __LINE__);
		}
	}
	nvic_enable_irq(uart->hardware->tx.dma_irqn);
}

void usbuart_usb_out_cb1(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[0], dev); (void) ep; }
void usbuart_usb_out_cb2(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[1], dev); (void) ep; }
void usbuart_usb_out_cb3(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[2], dev); (void) ep; }
void usbuart_usb_out_cb4(usbd_device *dev, uint8_t ep) { usbuart_usb_out_cb(&uarts[3], dev); (void) ep; }

void usbuart_usb_in_cb1(usbd_device *dev, uint8_t ep) { send_rx(&uarts[0]); (void) dev; (void) ep; }
void usbuart_usb_in_cb2(usbd_device *dev, uint8_t ep) { send_rx(&uarts[1]); (void) dev; (void) ep; }
void usbuart_usb_in_cb3(usbd_device *dev, uint8_t ep) { send_rx(&uarts[2]); (void) dev; (void) ep; }
void usbuart_usb_in_cb4(usbd_device *dev, uint8_t ep) { send_rx(&uarts[3]); (void) dev; (void) ep; }

void usbuart_set_line_coding(struct uart_t *dev, struct usb_cdc_line_coding *coding) {
	UART_TRACE(dev, __LINE__);
	dev->baud = coding->dwDTERate;
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
	switch(coding->bDataBits) {
	case 7:
		dev->bits = 8;
		break;
	case 8:
	default:
		if (dev->parity == USART_PARITY_NONE)
			dev->bits = 8;
		else
			dev->bits = 9;
		break;
	}
	/* need to make certain we can change at this time (is DMA finished, lines set at state saying no longer trying to send) */
	set_uart_parameters(dev);
}

void usbuart_set_control_line_state(struct uart_t *dev, uint16_t value) {
	UART_TRACE(dev, __LINE__);
	if (value & ACM_CTRL_RTS) {
		gpio_clear(dev->hardware->rts.port, dev->hardware->rts.pin);
		gpio_set(dev->hardware->dir.port, dev->hardware->dir.pin);
	} else {
		gpio_set(dev->hardware->rts.port, dev->hardware->rts.pin);
		gpio_clear(dev->hardware->dir.port, dev->hardware->dir.pin);
	}
	if (value & ACM_CTRL_DTR)
		gpio_clear(dev->hardware->dtr.port, dev->hardware->dtr.pin);
	else
		gpio_set(dev->hardware->dtr.port, dev->hardware->dtr.pin);
}

uint16_t uart_get_control_line_state(struct uart_t *dev) {
	uint16_t result = 0;

	if (!gpio_get(dev->hardware->cts.port, dev->hardware->cts.pin))
		result |= ACM_CTRL_CTS;
	
	if (!gpio_get(dev->hardware->dsr.port, dev->hardware->dsr.pin))
		result |= ACM_CTRL_DSR;

	if (!gpio_get(dev->hardware->dcd.port, dev->hardware->dcd.pin))
		result |= ACM_CTRL_DCD;

	if (!gpio_get(dev->hardware->ri.port, dev->hardware->ri.pin))
		result |= ACM_CTRL_RI;

	return result;
}

void usbuart_set_flow_control(struct uart_t *dev, uint16_t value) {
	UART_TRACE(dev, __LINE__);
	if (value)
		dev->flowcontrol = USART_FLOWCONTROL_CTS;
	else
		dev->flowcontrol = USART_FLOWCONTROL_NONE;

	usart_set_flow_control(dev->hardware->usart, dev->flowcontrol);
}

/*
 * Interrupt routines
 */

inline static void uart_TX_DMA_empty(struct uart_t *dev) {
	dma_disable_channel(dev->hardware->tx.dma, dev->hardware->tx.channel);
	usart_disable_tx_dma(dev->hardware->usart);
	UART_TRACE(dev, __LINE__);
	if (dma_get_interrupt_flag(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TCIF)) {
		// the transmit buffer is empty, clear the interupt flag
		dma_clear_interrupt_flags(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TCIF);
		if (dev->tx_num_to_send) {
			// There is data in the next buffer, set the DMA to send it
			dev->tx_curr_buffer_index = ((dev->tx_curr_buffer_index + 1) % NUM_TX_BUFFERS);
			dev->tx_dma_buffer_index = ((dev->tx_dma_buffer_index + 1) % NUM_TX_BUFFERS);
			dma_set_peripheral_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)&USART_DR(dev->hardware->usart));
			dma_set_memory_address(dev->hardware->tx.dma, dev->hardware->tx.channel, (uint32_t)(dev->buffers[dev->tx_dma_buffer_index]));
			dma_set_number_of_data(dev->hardware->tx.dma, dev->hardware->tx.channel, dev->tx_num_to_send);
			dev->tx_num_to_send = 0;
			usart_enable_tx_dma(dev->hardware->usart);
			dma_enable_channel(dev->hardware->tx.dma, dev->hardware->tx.channel);

			if (dev->tx_state == TX_FULL) {
				dev->tx_state = TX_WORKING;
				dev->tx_num_to_send = usbd_ep_read_packet(usbdev, dev->ep, dev->buffers[dev->tx_curr_buffer_index], CDCACM_PACKET_SIZE);
				UART_TRACE(dev, __LINE__);
			} else {
				dev->tx_state = TX_WORKING;
				UART_TRACE(dev, __LINE__);
			}
		} else {
			dev->tx_num_to_send = 0;
			dev->tx_state = TX_IDLE;

			schedule_ctrl_update(dev, true);
			UART_TRACE(dev, __LINE__);
		}
	}
	if (dma_get_interrupt_flag(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TEIF)) {
		dma_clear_interrupt_flags(dev->hardware->tx.dma, dev->hardware->tx.channel, DMA_TEIF);
		dev->tx_state = TX_ERROR;
		UART_TRACE(dev, __LINE__);
	}
}
void UART1_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[0]); }
void UART2_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[1]); }
void UART3_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[2]); }
void UART4_TX_DMA_ISR(void) { uart_TX_DMA_empty(&uarts[3]); }

void process_rx(struct uart_t *dev) {
	// get the number of characters read
	dev->num_read = RX_BUFFER_SIZE - DMA_CNDTR(dev->hardware->rx.dma, dev->hardware->rx.channel);
	// reset the DMA with the other buffer
	dma_set_number_of_data(dev->hardware->rx.dma, dev->hardware->rx.channel, RX_BUFFER_SIZE);
	if ((uint32_t)(dev->rx_buffer[0]) == DMA_CMAR(dev->hardware->rx.dma, dev->hardware->rx.channel)) {
		dev->rx_dma_index = 0;
		dma_set_memory_address(dev->hardware->rx.dma, dev->hardware->rx.channel, (uint32_t)(dev->rx_buffer[1]));
	} else {
		dev->rx_dma_index = 1;
		dma_set_memory_address(dev->hardware->rx.dma, dev->hardware->rx.channel, (uint32_t)(dev->rx_buffer[0]));
	}
	dma_enable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
	dev->rx_state |= RX_NEED_SERVICE;
	send_rx(dev);
}

void send_rx(struct uart_t *dev) {
	nvic_disable_irq(dev->hardware->timer_irqn);
	nvic_disable_irq(dev->hardware->rx.dma_irqn);
	nvic_disable_irq(dev->hardware->irqn);
	if (dev->rx_state & RX_NEED_SERVICE) {
		if (nt124_get_config()) {
			uint8_t reply_buf[sizeof(uint16_t) + RX_BUFFER_SIZE];
			uint16_t ctrl_state;
			uint8_t * rx_buffer = dev->rx_buffer[dev->rx_dma_index];
			int len = (int) dev->num_read;
			int i;

			dev->usb_in_tx_state = USB_TX_WORKING;
			ctrl_state = uart_get_control_line_state(dev);
			reply_buf[0] = (uint8_t)(ctrl_state & 0xff);
			reply_buf[1] = (uint8_t)(ctrl_state >> 8);
			
			for (i=0; i < len; i++) {
				reply_buf[sizeof(uint16_t) + i] = rx_buffer[i];
			}
			
			if(usbd_ep_write_packet(usbdev, dev->ep, reply_buf, len+2) == 0) {
				return;
			}

			ctrl_update_sent(dev, false);

			dev->usb_in_tx_state = USB_TX_IDLE;
		}

		// clear and disable the timer
		timer_clear_flag(dev->hardware->timer, TIM_SR_UIF);
		timer_disable_irq(dev->hardware->timer, TIM_DIER_UIE);

		dev->rx_state &= ~RX_NEED_SERVICE;
	}
	
	nvic_enable_irq(dev->hardware->timer_irqn);
	nvic_enable_irq(dev->hardware->rx.dma_irqn);
	nvic_enable_irq(dev->hardware->irqn);
}

// TODO: Need to add in something to check for overwriting a buffer that was not already serviced
inline static void uart_RX_DMA(struct uart_t *dev) {
	dma_disable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
	nvic_disable_irq(dev->hardware->timer_irqn);
	nvic_disable_irq(dev->hardware->rx.dma_irqn);
	nvic_disable_irq(dev->hardware->irqn);
	UART_TRACE(dev, __LINE__);
	if (dma_get_interrupt_flag(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TCIF)) {
		dma_clear_interrupt_flags(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TCIF);
		process_rx(dev);
		UART_TRACE(dev, __LINE__);
	}
	if (dma_get_interrupt_flag(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TEIF)) {
		// DMA error was encountered
		// TODO: Better handiling should be implemented and not just disable the uart
		dma_clear_interrupt_flags(dev->hardware->rx.dma, dev->hardware->rx.channel, DMA_TEIF);
		dev->rx_state |= RX_DMA_ERROR;
		usart_disable(dev->hardware->usart);
		UART_TRACE(dev, __LINE__);
	}
}
void UART1_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[0]); }
void UART2_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[1]); }
void UART3_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[2]); }
void UART4_RX_DMA_ISR(void) { uart_RX_DMA(&uarts[3]); }

// (re)start the timer for the RX delayed processing.
// Whenever a new character is received the timer starts over
// When the timer overflows the data in the DMA will be processed (Or when the DMA buffer is filled)
inline static void uart_RX(struct uart_t *dev) {
	uint32_t uart_sr = USART_SR(dev->hardware->usart);
	UART_TRACE(dev, __LINE__);
	if (uart_sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) {
		UART_TRACE(dev, __LINE__);
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
		nvic_disable_irq(dev->hardware->timer_irqn);
		// clear the flags if not already cleared
		timer_clear_flag(dev->hardware->timer, TIM_SR_UIF);
		USART_SR(dev->hardware->usart) &= ~USART_SR_RXNE;
		// (re)start timer
		timer_disable_counter(dev->hardware->timer);
		timer_set_counter(dev->hardware->timer, 0);
		timer_enable_counter(dev->hardware->timer);
		timer_enable_irq(dev->hardware->timer, TIM_DIER_UIE);
		// IRQ# for timer is always enabled
		nvic_enable_irq(dev->hardware->timer_irqn);
	}
	UART_TRACE(dev, __LINE__);
}
void UART1_RX_ISR(void) { uart_RX(&uarts[0]); }
void UART2_RX_ISR(void) { uart_RX(&uarts[1]); }
void UART3_RX_ISR(void) { uart_RX(&uarts[2]); }
void UART4_RX_ISR(void) { uart_RX(&uarts[3]); }

inline static void uart_RX_timer(struct uart_t *dev) {
	// stop RX
	dma_disable_channel(dev->hardware->rx.dma, dev->hardware->rx.channel);
	nvic_disable_irq(dev->hardware->rx.dma_irqn);
	nvic_disable_irq(dev->hardware->irqn);
	nvic_disable_irq(dev->hardware->timer_irqn);
	UART_TRACE(dev, __LINE__);
	process_rx(dev);
	UART_TRACE(dev, __LINE__);
}
void UART1_RX_TIMER(void) { uart_RX_timer(&uarts[0]); }
void UART2_RX_TIMER(void) { uart_RX_timer(&uarts[1]); }
void UART3_RX_TIMER(void) { uart_RX_timer(&uarts[2]); }
void UART4_RX_TIMER(void) { uart_RX_timer(&uarts[3]); }

/*
 * Interrupt routines for hardware control lines *
 */

/*
 * exti0 - uart2 CTS
 */
void exti0_isr(void) {
	exti_reset_request(EXTI0);
	schedule_ctrl_update(&uarts[1], false);
}

/*
 * exti1 - uart3 DSR
 */
void exti1_isr(void) {
	exti_reset_request(EXTI1);
	schedule_ctrl_update(&uarts[2], false);
}

/*
 * exti2 - uart4 CTS
 */
void exti2_isr(void) {
	exti_reset_request(EXTI2);
	schedule_ctrl_update(&uarts[3], false);
}

/*
 * exti3 - uart2 DSR
 */
void exti3_isr(void) {
	exti_reset_request(EXTI3);
	schedule_ctrl_update(&uarts[1], false);
}

/* exti4 - uart2 DCD */
void exti4_isr(void) {
	exti_reset_request(EXTI4);
	schedule_ctrl_update(&uarts[1], false);
}

/*
 * exti5 - uart2 RI
 * exti6 - uart1 DCD
 * exti7 - uart1 DSR
 * exti8 - uart1 CTS
 * exti9 - uart4 DCD
 */
void exti9_5_isr(void) {
	uint32_t flags = exti_get_flag_status(EXTI5 | EXTI6 | EXTI7 | EXTI8 | EXTI9);
	exti_reset_request(EXTI5 | EXTI6 | EXTI7 | EXTI8 | EXTI9);
	
	if (flags & uarts[1].hardware->ri.pin)
		schedule_ctrl_update(&uarts[1], false);

	if (	flags & uarts[0].hardware->dcd.pin ||
		flags & uarts[0].hardware->dsr.pin ||
		flags & uarts[0].hardware->cts.pin)
		schedule_ctrl_update(&uarts[0], false);

	if (flags & uarts[3].hardware->dcd.pin)
		schedule_ctrl_update(&uarts[1], false);
}

/*
 * exti12 - uart3 DCD
 * exti13 - uart3 CTS
 * exti14 - uart4 DSR
 * exti15 - uart3 RI
 */
void exti15_10_isr(void) {
	uint32_t flags = exti_get_flag_status(EXTI12 | EXTI13 | EXTI14 | EXTI15);
	exti_reset_request(EXTI12 | EXTI13 | EXTI14 | EXTI15);

	if (    flags & uarts[2].hardware->dcd.pin ||
		flags & uarts[2].hardware->cts.pin ||
		flags & uarts[2].hardware->ri.pin)
		schedule_ctrl_update(&uarts[2], false);
	
	if (flags & uarts[3].hardware->dsr.pin)
		schedule_ctrl_update(&uarts[3], false);
	
}

uint8_t usbuart_get_txempty(struct uart_t *dev)
{
	if (dev->tx_empty)
		return 1;
	else
		return 0;
}
