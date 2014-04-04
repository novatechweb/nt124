#include "platform_uart.h"

#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

/* UART1 : ACM PORT1 */
struct platform_uart_t uart1 = {
	.usart = USART1,
	.irqn = NVIC_USART1_IRQ,
	.timer = TIM2,
	.timer_irqn = NVIC_TIM2_IRQ,
	.rx =  {	// RX
		.port = GPIO_BANK_USART1_RX,
		.pin = GPIO_USART1_RX,
		.dma_irqn = NVIC_DMA1_CHANNEL5_IRQ,
		.dma = DMA1,
		.channel = DMA_CHANNEL5,
	},
	.cts = {	// CTS
		.port = GPIOC,
		.pin = GPIO8,
		.irqn = 0xFF,
	},
	.dsr = {	// DSR
		.port = GPIOC,
		.pin = GPIO7,
		.irqn = 0xFF,
	},
	.dcd = {	// DCD
		.port = GPIOC,
		.pin = GPIO6,
		.irqn = 0xFF,
	},
	.ri =  {	// RI
		.port = GPIOB,
		.pin = GPIO0,
		.irqn = 0xFF,
	},
	.tx =  {	// TX
		.port = GPIO_BANK_USART1_TX,
		.pin = GPIO_USART1_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		.dma_irqn = NVIC_DMA1_CHANNEL4_IRQ,
		.dma = DMA1,
		.channel = DMA_CHANNEL4,
	},
	.rts = {	// RTS
		.port = GPIOC,
		.pin = GPIO9,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO12,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
};

/* UART2 : ACM PORT4 */
struct platform_uart_t uart2 = {
	.usart = USART2,
	.irqn = NVIC_USART2_IRQ,
	.timer = TIM3,
	.timer_irqn = NVIC_TIM3_IRQ,
	.rx =  {	// RX
		.port = GPIO_BANK_USART2_RX,
		.pin = GPIO_USART2_RX,
		.dma_irqn = NVIC_DMA1_CHANNEL6_IRQ,
		.dma = DMA1,
		.channel = DMA_CHANNEL6,
	},
	.cts = {	// CTS
		.port = GPIO_BANK_USART2_CTS,
		.pin = GPIO_USART2_CTS,
		.irqn = 0xFF,
	},
	.dsr = {	// DSR
		.port = GPIOC,
		.pin = GPIO3,
		.irqn = 0xFF,
	},
	.dcd = {	// DCD
		.port = GPIOA,
		.pin = GPIO4,
		.irqn = 0xFF,
	},
	.ri =  {	// RI
		.port = GPIOA,
		.pin = GPIO5,
		.irqn = 0xFF,
	},
	.tx =  {	// TX
		.port = GPIO_BANK_USART2_TX,
		.pin = GPIO_USART2_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		.dma_irqn = NVIC_DMA1_CHANNEL7_IRQ,
		.dma = DMA1,
		.channel = DMA_CHANNEL7,
	},
	.rts = {	// RTS
		.port = GPIO_BANK_USART2_RTS,
		.pin = GPIO_USART2_RTS,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	//	.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO2,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
};

/* UART3 : ACM PORT3 */
struct platform_uart_t uart3 = {
	.usart = USART3,
	.irqn = NVIC_USART3_IRQ,
	.timer = TIM4,
	.timer_irqn = NVIC_TIM4_IRQ,
	.rx =  {	// RX
		.port = GPIO_BANK_USART3_RX,
		.pin = GPIO_USART3_RX,
		.dma_irqn = NVIC_DMA1_CHANNEL3_IRQ,
		.dma = DMA1,
		.channel = DMA_CHANNEL3,
	},
	.cts = {	// CTS
		.port = GPIO_BANK_USART3_CTS,
		.pin = GPIO_USART3_PR_CTS,
		.irqn = 0xFF,
	},
	.dsr = {	// DSR
		.port = GPIOB,
		.pin = GPIO1,
		.irqn = 0xFF,
	},
	.dcd = {	// DCD
		.port = GPIOB,
		.pin = GPIO12,
		.irqn = 0xFF,
	},
	.ri  = {	// RI
		.port = GPIOB,
		.pin = GPIO15,
		.irqn = 0xFF,
	},
	.tx =  {	// TX
		.port = GPIO_BANK_USART3_TX,
		.pin = GPIO_USART3_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		.dma_irqn = NVIC_DMA1_CHANNEL2_IRQ,
		.dma = DMA1,
		.channel = DMA_CHANNEL2,
	},
	.rts = {	// RTS
		.port = GPIO_BANK_USART3_RTS,
		.pin = GPIO_USART3_RTS,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	//	.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO5,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
};

/* UART4 : ACM PORT2 */
struct platform_uart_t uart4 = {
	.usart = UART4,
	.irqn = NVIC_UART4_IRQ,
	.timer = TIM5,
	.timer_irqn = NVIC_TIM5_IRQ,
	.rx =  {	// RX
		.port = GPIO_BANK_UART4_RX,
		.pin = GPIO_UART4_RX,
		.dma_irqn = NVIC_DMA2_CHANNEL3_IRQ,
		.dma = DMA2,
		.channel = DMA_CHANNEL3,
	},
	.cts = {	// CTS
		.port = GPIOD,
		.pin = GPIO2,
		.irqn = 0xFF,
	},
	.dsr = {	// DSR
		.port = GPIOC,
		.pin = GPIO14,
		.irqn = 0xFF,
	},
	.dcd = {	// DCD
		.port = GPIOB,
		.pin = GPIO9,
		.irqn = 0xFF,
	},
	.ri =  {	// RI
		.port = GPIOB,
		.pin = GPIO8,
		.irqn = 0xFF,
	},
	.tx =  {	// TX
		.port = GPIO_BANK_UART4_TX,
		.pin = GPIO_UART4_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		.dma_irqn = NVIC_DMA2_CHANNEL4_5_IRQ,
		.dma = DMA2,
		.channel = DMA_CHANNEL5,
	},
	.rts = {	// RTS
		.port = GPIOB,
		.pin = GPIO6,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOB,
		.pin = GPIO5,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
};
