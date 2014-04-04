#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#ifdef STM32F1
  #include "platform/STM32F1/platform_STM32F1.h"
  #include "platform/STM32F1/platform_uart.h"
  #include "platform/STM32F1/platform_usb.h"
#endif

#define CDCACM_PACKET_SIZE 	32

#define NUM_TX_BUFFERS 2
#define TX_BUFFER_SIZE CDCACM_PACKET_SIZE
#define RX_BUFFER_SIZE CDCACM_PACKET_SIZE

/*** Interrupt priorities ***/
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB         ( 2 << 4)

#define IRQ_PRI_UART        (1 << 4)
#define IRQ_PRI_UART_TIM    (1 << 4)
#define IRQ_PRI_UART_RX_DMA (1 << 3)
#define IRQ_PRI_UART_TX_DMA (1 << 5)

#endif /* __PLATFORM_H_ */
