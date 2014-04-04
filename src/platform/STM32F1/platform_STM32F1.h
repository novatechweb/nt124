/*  Pin Assignment : STM32F103 RCT6
 *  Pin         Assignment
 *  31  VSS1    Ground
 *  32  VDD1    3.3V
 *  47  VSS2    Ground
 *  48  VDD2    3.3V
 *  63  VSS3    Ground
 *  64  VDD3    3.3V
 *  18  VSS4    Ground
 *  19  VDD4    3.3V
 *  01  VBAT    3.3V
 *  12  VSSa    Ground
 *  13  VDDa    3.3V
 *  
 *  60  BOOT0   Boot0
 *  07  RESET   JTAG/Reset
 *  05  PD0     OscIn
 *  06  PD1     OscOut
 *  54  PD2     CTS4
 *  
 *  14  PA0     CTS2
 *  15  PA1     RTS2
 *  16  PA2     TX2
 *  17  PA3     RX2
 *  20  PA4     DCD2
 *  21  PA5     RI2
 *  22  PA6     I/O
 *  23  PA7     I/O
 *  41  PA8     USB-ENUM
 *  42  PA9     TX1
 *  43  PA10    RX1
 *  44  PA11    USB D-
 *  45  PA12    USB D+
 *  46  PA13    JTAG
 *  49  PA14    JTAG
 *  50  PA15    JTAG
 *  
 *  26  PB0     RI1
 *  27  PB1     DSR3
 *  28  PB2     Boot1/DFU
 *  55  PB3     JTAG
 *  56  PB4     JTAG
 *  57  PB5     DTR4
 *  58  PB6     RTS4
 *  59  PB7     I/O
 *  61  PB8     RI4
 *  62  PB9     DCD4
 *  29  PB10    TX3
 *  30  PB11    RX3
 *  33  PB12    DCD3
 *  34  PB13    CTS3
 *  35  PB14    RTS3
 *  36  PB15    RI3
 *  
 *  08  PC0     I/O
 *  09  PC1     I/O
 *  10  PC2     DTR2
 *  11  PC3     DSR2
 *  24  PC4     I/O
 *  25  PC5     DTR3
 *  37  PC6     DCD1
 *  38  PC7     DSR1
 *  39  PC8     CTS1
 *  40  PC9     RTS1
 *  51  PC10    TX4
 *  52  PC11    RX4
 *  53  PC12    DTR1
 *  02  PC13    I/O
 *  03  PC14    DSR4
 *  04  PC15    I/O
 */


#ifndef __PLATFORM_STM32F1_H_
#define __PLATFORM_STM32F1_H_

#include <stdint.h>
#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/usb/usbd.h>


void platform_init(void);
//void assert_boot_pin(void);


#ifdef INLINE_GPIO
/* Apply soime optimizations that are in the library libopencm3 */
static inline void _gpio_set(uint32_t gpioport, uint16_t gpios) {
        GPIO_BSRR(gpioport) = gpios;
}
#define gpio_set _gpio_set

static inline void _gpio_clear(uint32_t gpioport, uint16_t gpios) {
        GPIO_BRR(gpioport) = gpios;
}
#define gpio_clear _gpio_clear

static inline uint16_t _gpio_get(uint32_t gpioport, uint16_t gpios) {
        return (uint16_t)GPIO_IDR(gpioport) & gpios;
}
#define gpio_get _gpio_get
#endif /* INLINE_GPIO */


#endif /* __PLATFORM_STM32F1_H_ */
