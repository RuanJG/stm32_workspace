this project is for stm32f103rct6 jostick, 6 channels , the emiter may use uart2 or spi1 ( Just chose one), the pin use under line:



spi

/* SPI1 BANK */
#define GPIO_BANK_SPI1_NSS		GPIOA		/* PA4 */
#define GPIO_BANK_SPI1_SCK		GPIOA		/* PA5 */
#define GPIO_BANK_SPI1_MISO		GPIOA		/* PA6 */
#define GPIO_BANK_SPI1_MOSI		GPIOA		/* PA7 */


uart3:
/* USART3 GPIO */
#define GPIO_USART3_TX			GPIO10		/* PB10 */
#define GPIO_USART3_RX			GPIO11		/* PB11 */
#define GPIO_USART3_CK			GPIO12		/* PB12 */
#define GPIO_USART3_CTS			GPIO13		/* PB13 */
#define GPIO_USART3_RTS			GPIO14		/* PB14 */


uart2:
/* USART2 GPIO */
#define GPIO_USART2_CTS			GPIO0		/* PA0 */
#define GPIO_USART2_RTS			GPIO1		/* PA1 */
#define GPIO_USART2_TX			GPIO2		/* PA2 */
#define GPIO_USART2_RX			GPIO3		/* PA3 */
#define GPIO_USART2_CK			GPIO4		/* PA4 */


uart1:
#define GPIO_USART1_TX			GPIO9		/* PA9 */
#define GPIO_USART1_RX			GPIO10		/* PA10 */

#define GPIO_USART1_RE_TX		GPIO6		/* PB6 */
#define GPIO_USART1_RE_RX		GPIO7		/* PB7 */


adc:

  ADC1/2:                   ADC3:
  C0  -> PA0                C0  -> PA0
  C1  -> PA1                C1  -> PA1
  C2  -> PA2                C2  -> PA2
  C3  -> PA3                C3  -> PA3
  C4  -> PA4                C4  -> PF6
  C5  -> PA5                C5  -> PF7
  C6  -> PA6                C6  -> PF8
  C7  -> PA7                C7  -> PF9
  C8  -> PB0                C8  -> PF10
  C9  -> PB1
  C10 -> PC0                C10 -> PC0
  C11 -> PC1                C11 -> PC1
  C12 -> PC2                C12 -> PC2
  C13 -> PC3                C13 -> PC3
  C14 -> PC4
  C15 -> PC5

AD1 : pc0 pc1 pc2 pc3 pc4 pc5


board used: 
A11 a12 a8
b6 b7
c8 c9 c12  c11 c10  c13 
d2


