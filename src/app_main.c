#include <mcu_periph/uart.h>
#include <mcu.h>
#include "mcu_periph/sys_time.h"

#define COPTER_UART &uart1
#define TELEM_UART &uart2
#define REMOTE_4G_UART &uart3
//#define SBUS_PPM_UART &uart3

#define CONSOLE_UART &uart1

#define COPTER_COM_ID MAVLINK_COMM_0
#define REMOTE_4G_COM_ID MAVLINK_COMM_1
#define TELEM_COM_ID MAVLINK_COMM_2
//#define SBUS_PPM_COM_ID MAVLINK_COMM_1

#define DEBUG_APP 1
#define log(format, ...) if( DEBUG_APP ) printf(format, ## __VA_ARGS__)

//#include "uart.c"
int do_uart_wirte(struct uart_periph *uart,uint8_t *buff,int len)
{
	int i;
	for(i = 0; i< len; i++)
	{
		uart_put_byte(uart, buff[len]);		
	}
	return len;
}
int do_uart_read(struct uart_periph *uart, uint8_t *buff ,int len)
{
	int data_len;
	int i;
	data_len = uart_char_available(uart);
	data_len = data_len > len ? len:data_len;
	for( i=0; i< data_len ; i++)
		buff[i]= uart_getch(uart);
	return data_len;
}
//for printf
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		return do_uart_wirte(CONSOLE_UART, (uint8_t*)ptr,len);
	}
	//errno = EIO;
	return -1;
}


void do_echo(struct uart_periph *uart)
{
	uint8_t buff[1024];
	int len,res;
	int i;

	len = uart_char_available(uart);
	len = len > 1024? 1024:len;
	for(i = 0; i< len; i++)
	{
		uart_put_byte(uart, uart_getch(uart));		
	}
}

void delay_ms(int ms)
{
	sys_time_usleep(ms*1000);		
}



void do_copy_uart_data_to_other_uart(struct uart_periph *uarts ,struct uart_periph *uartd)
{
	int data_len,i;
	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++)
			uart_put_byte(uartd, uart_getch(uarts) );
	}
}
void do_copy_uart_data_to_two_uart( struct uart_periph *uarts ,struct uart_periph *uartd1, struct uart_periph *uartd2)
{
	int data_len,i;
	uint8_t c;
	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++){
			c = uart_getch(uarts);
			uart_put_byte(uartd1, c );
			uart_put_byte(uartd2, c );
		}
	}
}

void loop_4g_and_telem_to_copter()
{
	do_copy_uart_data_to_two_uart(COPTER_UART,REMOTE_4G_UART,TELEM_UART);
	do_copy_uart_data_to_other_uart(REMOTE_4G_UART,COPTER_UART);
	do_copy_uart_data_to_other_uart(TELEM_UART,COPTER_UART);
}
void loop_telem_and_sbusppm_to_copter()
{
	;//mavlink_parse_char
}

void setup()
{
	mcu_init(); //define PERIPHERALS_AUTO_INIT in makefile , and will init preiph auto in mcu_init(), like uartx_init()

	//uart define config in makefile and uart_arch.c 
  	//uart_periph_set_mode(&uart1, USE_UART1_TX, USE_UART1_RX, UART1_HW_FLOW_CONTROL);
  	//uart_periph_set_bits_stop_parity(&uart1, UART1_BITS, UART1_STOP, UART1_PARITY);
  	//uart_periph_set_baudrate(&uart1, UART1_BAUD);
}
void loop()
{
	static int s = 0;
	log("start delay %d s\n",s);
	delay_ms(1000);
	s++;
	loop_4g_and_telem_to_copter();
	//do_echo(&uart1);
	//do_echo(&uart2);
	//do_echo(&uart3);
}

int main(void)
{
	setup();
	while(1){
		loop();
	}
	return 0;
}

