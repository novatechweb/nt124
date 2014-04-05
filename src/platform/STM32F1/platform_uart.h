#ifndef __PLATFORM_UART_H_
#define __PLATFORM_UART_H_

#include <stdint.h>

#define UART1_RX_TIMER   tim2_isr
#define UART2_RX_TIMER   tim3_isr
#define UART3_RX_TIMER   tim4_isr
#define UART4_RX_TIMER   tim5_isr

#define UART1_RX_ISR     usart1_isr
#define UART2_RX_ISR     usart2_isr
#define UART3_RX_ISR     usart3_isr
#define UART4_RX_ISR     uart4_isr

#define UART1_RX_DMA_ISR dma1_channel5_isr
#define UART2_RX_DMA_ISR dma1_channel6_isr
#define UART3_RX_DMA_ISR dma1_channel3_isr
#define UART4_RX_DMA_ISR dma2_channel3_isr

#define UART1_TX_DMA_ISR dma1_channel4_isr
#define UART2_TX_DMA_ISR dma1_channel7_isr
#define UART3_TX_DMA_ISR dma1_channel2_isr
#define UART4_TX_DMA_ISR dma2_channel4_5_isr

struct platform_RX_t {
	const uint32_t port;
	const uint32_t dma;
	const uint16_t pin;
	const uint8_t channel;
	const uint8_t dma_irqn;
};
struct platform_TX_t {
	const uint32_t port;
	const uint32_t dma;
	const uint16_t pin;
	const uint8_t cnf;
	const uint8_t dma_irqn;
	const uint8_t channel;
};
struct platform_input_pins_t {
	const uint32_t port;
	const uint16_t pin;
	const uint8_t irqn;
};
struct platform_output_pins_t {
	const uint32_t port;
	const uint16_t pin;
	const uint8_t cnf;
};
struct platform_uart_t {
	const uint32_t usart;
	const uint32_t timer;
	const uint8_t timer_irqn;
	const uint8_t irqn;
	struct platform_RX_t rx;
	struct platform_input_pins_t cts;
	struct platform_input_pins_t dsr;
	struct platform_input_pins_t dcd;
	struct platform_input_pins_t ri;
	struct platform_TX_t tx;
	struct platform_output_pins_t rts;
	struct platform_output_pins_t dtr;
};

void uart_platform_init(void);

#endif /* __PLATFORM_UART_H_ */
