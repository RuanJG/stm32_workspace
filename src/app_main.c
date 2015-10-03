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
	mcu_init();
	while(1){
		do_echo(&uart1);
		__asm__("nop");
	}
	return 0;
}

