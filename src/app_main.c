#include <mcu_periph/uart.h>
#include <mcu.h>


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

int main(void)
{
	int i;
	mcu_init(); //define PERIPHERALS_AUTO_INIT in makefile , and will init preiph auto in mcu_init(), like uartx_init()
	while(1){
		//do_echo(&uart3);
		do_echo(&uart1);
		do_echo(&uart2);
		do_echo(&uart3);
		__asm__("nop");
	}
	return 0;
}

