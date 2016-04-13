

/* Timer3 GPIO */
#define GPIO_TIM3_FR_CH1		GPIO6		/* PC6 */
#define GPIO_TIM3_FR_CH2		GPIO7		/* PC7 */
#define GPIO_TIM3_FR_CH3		GPIO8		/* PC8 */
#define GPIO_TIM3_FR_CH4		GPIO9		/* PC9 */

/* Timer4 GPIO */
#define GPIO_TIM4_CH1			GPIO6		/* PB6 */
#define GPIO_TIM4_CH2			GPIO7		/* PB7 */
#define GPIO_TIM4_CH3			GPIO8		/* PB8 */
#define GPIO_TIM4_CH4			GPIO9		/* PB9 */

/* Timer2 GPIO */
#define GPIO_TIM2_FR_CH1_ETR		GPIO15		/* PA15 */
#define GPIO_TIM2_FR_CH2		GPIO3		/* PB3 */
#define GPIO_TIM2_FR_CH3		GPIO10		/* PB10 */
#define GPIO_TIM2_FR_CH4		GPIO11		/* PB11 */

/* Timer5 GPIO */
#define GPIO_TIM5_CH1			GPIO0		/* PA0 */
#define GPIO_TIM5_CH2			GPIO1		/* PA1 */
#define GPIO_TIM5_CH3			GPIO2		/* PA2 */
#define GPIO_TIM5_CH4			GPIO3		/* PA3 */


#define GPIO_USART1_TX			GPIO9		/* PA9 */
#define GPIO_USART1_RX			GPIO10		/* PA10 */

#define GPIO_USART3_PR_TX		GPIO10		/* PC10 */
#define GPIO_USART3_PR_RX		GPIO11		/* PC11 */
#define GPIO_USART3_PR_CK		GPIO12		/* PC12 */
#define GPIO_USART3_PR_CTS		GPIO13		/* PB13 */
#define GPIO_USART3_PR_RTS		GPIO14		/* PB14 */

/* SPI1 GPIO */
#define GPIO_SPI1_NSS			GPIO4		/* PA4 */
#define GPIO_SPI1_SCK			GPIO5		/* PA5 */
#define GPIO_SPI1_MISO			GPIO6		/* PA6 */
#define GPIO_SPI1_MOSI			GPIO7		/* PA7 */

/* SPI2 GPIO */
#define GPIO_SPI2_NSS			GPIO12		/* PB12 */
#define GPIO_SPI2_SCK			GPIO13		/* PB13 */
#define GPIO_SPI2_MISO			GPIO14		/* PB14 */
#define GPIO_SPI2_MOSI			GPIO15		/* PB15 */


pwm : tim3[1-4ch]=chan[0,3], tim4[1-4ch]=chan[4-7] ...

2.4G (radio) : use spi or uart3 
if (read rc from uart){
	use uart1 
}else{
	subs output : use uart1
}

if ( ! tim5 ){
	use uart2 is valiable
}
