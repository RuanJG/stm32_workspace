#include "mcu_periph/uart.h"


int main(void)
{
	while(1){
		loop();
		__asm__("nop");
	}
	return 0;
}

