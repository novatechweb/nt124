#include "platform_uart.h"

#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
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
		.exti = EXTI8,
		.irqn = NVIC_EXTI9_5_IRQ,
	},
	.dsr = {	// DSR
		.port = GPIOC,
		.pin = GPIO7,
		.exti = EXTI7,
		.irqn = NVIC_EXTI9_5_IRQ,
	},
	.dcd = {	// DCD
		.port = GPIOC,
		.pin = GPIO6,
		.exti = EXTI6,
		.irqn = NVIC_EXTI9_5_IRQ,
	},
	.ri =  {	// RI
		.port = GPIOB,
		.pin = GPIO0,
		.exti = 0xFF,	// just a filler value that does not corispond to an exti
		.irqn = NVIC_IRQ_COUNT,
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
		.exti = EXTI0,
		.irqn = NVIC_EXTI0_IRQ,
	},
	.dsr = {	// DSR
		.port = GPIOC,
		.pin = GPIO3,
		.exti = EXTI3,
		.irqn = NVIC_EXTI3_IRQ,
	},
	.dcd = {	// DCD
		.port = GPIOA,
		.pin = GPIO4,
		.exti = EXTI4,
		.irqn = NVIC_EXTI4_IRQ,
	},
	.ri =  {	// RI
		.port = GPIOA,
		.pin = GPIO5,
		.exti = EXTI5,
		.irqn = NVIC_EXTI9_5_IRQ,
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
		.exti = EXTI13,
		.irqn = NVIC_EXTI15_10_IRQ,
	},
	.dsr = {	// DSR
		.port = GPIOB,
		.pin = GPIO1,
		.exti = EXTI1,
		.irqn = NVIC_EXTI1_IRQ,
	},
	.dcd = {	// DCD
		.port = GPIOB,
		.pin = GPIO12,
		.exti = EXTI12,
		.irqn = NVIC_EXTI15_10_IRQ,
	},
	.ri  = {	// RI
		.port = GPIOB,
		.pin = GPIO15,
		.exti = EXTI15,
		.irqn = NVIC_EXTI15_10_IRQ,
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
		.exti = EXTI2,
		.irqn = NVIC_EXTI2_IRQ,
	},
	.dsr = {	// DSR
		.port = GPIOC,
		.pin = GPIO14,
		.exti = EXTI14,
		.irqn = NVIC_EXTI15_10_IRQ,
	},
	.dcd = {	// DCD
		.port = GPIOB,
		.pin = GPIO9,
		.exti = EXTI9,
		.irqn = NVIC_EXTI9_5_IRQ,
	},
	.ri =  {	// RI
		.port = GPIOB,
		.pin = GPIO8,
		.exti = 0xFF,	// just a filler value that does not corispond to an exti
		.irqn = NVIC_IRQ_COUNT,
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

void uart_platform_init(void) {
	// reset the hardware
	rcc_peripheral_reset(&RCC_APB1RSTR,
		RCC_APB1RSTR_UART4RST | RCC_APB1RSTR_USART3RST | RCC_APB1RSTR_USART2RST |
		RCC_APB1RSTR_TIM5RST | RCC_APB1RSTR_TIM4RST |
		RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST);
	rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_USART1RST | RCC_APB2RSTR_TIM1RST | RCC_APB2RSTR_AFIORST);
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
	// Alternate function IO clock enable
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
}
