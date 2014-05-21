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
#define NUM_RX_BUFFERS 2
#define RX_BUFFER_SIZE (CDCACM_PACKET_SIZE-3)

/*** Interrupt priorities ***/
/* Interrupt priorities.  Low numbers are high priority.
 * ((1 << 7) + ?) are assigned to the system interrupts.
 */
#define IRQ_PRI_SYSTICK     (0 << 4)
#define IRQ_PRI_UART_TX_DMA (1 << 4)
#define IRQ_PRI_EXT_INT     (2 << 4)
#define IRQ_PRI_UART_RX_DMA (3 << 4)
#define IRQ_PRI_UART_TIM    (4 << 4)
#define IRQ_PRI_USB         (5 << 4)
#define IRQ_PRI_UART        (6 << 4)

#endif /* __PLATFORM_H_ */
