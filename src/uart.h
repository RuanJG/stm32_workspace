#ifndef __RUAN_UART_HEAD
#define __RUAN_UART_HEAD

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define USART1_USE_IRQ
#define USART2_USE_IRQ
#define USART3_USE_IRQ

#define RX_BUFF_SIZE 1024
typedef struct __usart_buffer
{
	uint8_t irq_rx_buffer[RX_BUFF_SIZE];
	uint16_t irq_rx_len;
	uint8_t rx_buffer[RX_BUFF_SIZE];
	uint16_t rx_len;
	uint8_t buffer_overflow;
	uint8_t id ;
	uint32_t usart;

}usart_buffer_t;
int usart1_setup(uint32_t baud,uint32_t bit, uint32_t stopbit, uint32_t parity, uint32_t mode);
int usart2_setup(uint32_t baud,uint32_t bit, uint32_t stopbit, uint32_t parity, uint32_t mode);
int usart3_setup(uint32_t baud,uint32_t bit, uint32_t stopbit, uint32_t parity, uint32_t mode);
int do_read_usart(int id, uint8_t *buffer, int len);
int do_write_usart(int id, uint8_t *buffer, int len);
int  do_usart_buffer_check(int id, int init);

#endif
