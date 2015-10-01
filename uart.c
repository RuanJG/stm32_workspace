#include "uart.h"
#include "app_main.h"

static usart_buffer_t usart1_buffer;
static usart_buffer_t usart2_buffer;
static usart_buffer_t usart3_buffer;


//for printf
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART1, ptr[i]);
		return i;
	}
	//errno = EIO;
	return -1;
}



static void do_usart_buffer_init(usart_buffer_t * buff, int id)
{
	buff->irq_rx_len = 0;
	buff->rx_len = 0;
	buff->id = id;
	buff->buffer_overflow = 0;
	if( id == 1 )
		buff->usart = USART1;
	if( id == 2 )
		buff->usart = USART2;
	if( id == 3 )
		buff->usart = USART3;
}
int  do_usart_buffer_check(int id, int init)
{
	usart_buffer_t *buff ;
	if( id == 1 ) buff = &usart1_buffer;
	else if( id == 2 ) buff = &usart2_buffer;
	else if( id == 3 ) buff = &usart3_buffer;
	else return -1;

	if( buff->buffer_overflow == 1 )
	{
		printf("usart%d buffer overflow !!!!\n",buff->id); 
		if( init == 1 )
			do_usart_buffer_init(buff, buff->id);
		return 1;
	}
	return 0;
}
static void do_usart_buffer_irq_read(usart_buffer_t *buff)
{
	uint8_t data = 'A';
	int rx_len;
	int irq_len;
	uint8_t *irq_buff;
	uint8_t *rx_buff;
	int i;

	irq_len = buff->irq_rx_len;  
	rx_len = buff->rx_len;  
	irq_buff = buff->irq_rx_buffer;
	rx_buff = buff->rx_buffer;
	if( buff->buffer_overflow == 1 ){
		return;
	}else{
		data = usart_recv(buff->usart);

		if( irq_len < RX_BUFF_SIZE){
			irq_buff[irq_len++]= data;
			if(rx_len == 0 )
			{
				for( i = 0 ; i< irq_len; i++)
					rx_buff[i] = irq_buff[i];
				buff->rx_len = irq_len;
				buff->irq_rx_len = 0;
			}else{
				buff->irq_rx_len = irq_len;
			}
		}else{
			if(rx_len == 0 )
			{
				for( i = 0 ; i< irq_len; i++)
					rx_buff[i] = irq_buff[i];
				buff->rx_len = irq_len;
				buff->irq_rx_len = 0;
				irq_buff[buff->irq_rx_len++] = data;
			}else{
				buff->buffer_overflow =1;
			}
			
		}
	}
}

int do_read_usart(int id, uint8_t *buffer, int len)
{
	int i,cp_len;
	usart_buffer_t *buff ;
	if( id == 1 ) buff = &usart1_buffer;
	else if( id == 2 ) buff = &usart2_buffer;
	else if( id == 3 ) buff = &usart3_buffer;
	else return -1;

	cp_len = len < buff->rx_len ? len:buff->rx_len;
	memcpy(buffer, buff->rx_buffer, cp_len);
	buff->rx_len -= cp_len;
	if( buff->rx_len <0 ) buff->rx_len = 0;
	return cp_len;
}
int do_write_usart(int id, uint8_t *buffer, int len)
{
	int i,cp_len;
	usart_buffer_t *buff ;
	if( id == 1 ) buff = &usart1_buffer;
	else if( id == 2 ) buff = &usart2_buffer;
	else if( id == 3 ) buff = &usart3_buffer;
	else return -1;

	for( i =0; i< len; i++ )
	{
		usart_send_blocking(buff->usart, buffer[i]);
	}
	return len;
}
static void usart_irq_handler(int id)
{
	usart_buffer_t *buff ;
	if( id == 1 ) buff = &usart1_buffer;
	else if( id == 2 ) buff = &usart2_buffer;
	else if( id == 3 ) buff = &usart3_buffer;
	else return ;

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(buff->usart) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(buff->usart) & USART_SR_RXNE) != 0)) {
		/* Retrieve the data from the peripheral. */
		do_usart_buffer_irq_read(buff);

	}
}

int usart1_setup(uint32_t baud,uint32_t bit, uint32_t stopbit, uint32_t parity, uint32_t mode)
{//return id
	int id = 1;
	do_usart_buffer_init(&usart1_buffer, id);
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, baud);//115200
	usart_set_databits(USART1, bit);//8
	usart_set_stopbits(USART1, stopbit);//USART_STOPBITS_1);
	usart_set_parity(USART1, parity);//USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, mode);//USART_MODE_TX_RX);

	#ifdef USART1_USE_IRQ
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;
	#endif

	/* Finally enable the USART. */
	usart_enable(USART1);
	return id;
}



int usart2_setup(uint32_t baud,uint32_t bit, uint32_t stopbit, uint32_t parity, uint32_t mode)
{//return id
	int id = 2;
	do_usart_buffer_init(&usart2_buffer, id);
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART2);

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART2_IRQ);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, baud);//115200
	usart_set_databits(USART2, bit);//8
	usart_set_stopbits(USART2, stopbit);//USART_STOPBITS_1);
	usart_set_parity(USART2, parity);//USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART2, mode);//USART_MODE_TX_RX);

	#ifdef USART2_USE_IRQ
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART2) |= USART_CR1_RXNEIE;
	#endif

	/* Finally enable the USART. */
	usart_enable(USART2);
	return id;
}

int usart3_setup(uint32_t baud,uint32_t bit, uint32_t stopbit, uint32_t parity, uint32_t mode)
{//return id
	int id = 3;
	do_usart_buffer_init(&usart3_buffer, id);
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART3);

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, baud);//115200
	usart_set_databits(USART3, bit);//8
	usart_set_stopbits(USART3, stopbit);//USART_STOPBITS_1);
	usart_set_parity(USART3, parity);//USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART3, mode);//USART_MODE_TX_RX);

	#ifdef USART3_USE_IRQ
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART3) |= USART_CR1_RXNEIE;
	#endif

	/* Finally enable the USART. */
	usart_enable(USART3);
	return id;
}


#ifdef USART1_USE_IRQ
void usart1_isr(void)
{
	usart_irq_handler(1);
}
#endif
#ifdef USART2_USE_IRQ
void usart2_isr(void)
{
	usart_irq_handler(2);
}
#endif
#ifdef USART3_USE_IRQ
void usart3_isr(void)
{
	usart_irq_handler(3);
}
#endif
