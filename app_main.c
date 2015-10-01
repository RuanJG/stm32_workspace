/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */



#include "app_main.h"
#include "uart.h"
#include "delay.h"


int usart1_id ;
int usart2_id ;
int usart3_id ;



void led_setup(void);
void heart_led_triggle();





static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

void do_echo(int id)
{
	uint8_t buff[1024];
	int len,res;

	len = do_read_usart(id , buff, 1024);
	if( len > 0 ){
		do_write_usart(id,buff,len);
		heart_led_triggle();
	}
}
int do_recive_and_handle_radio_message(int id)
{//id = MAVLINK_COMM_0 MAVLINK_COMM_1 MAVLINK_COMM_2
	uint8_t buff[1024];
	int len,i,res, recived,com;
	mavlink_message_t msg;
	mavlink_status_t status;

	com = id -1;

	len = do_read_usart(id , buff, 1024);
	if( len > 0 ){
		//to the copter directly
		do_write_usart(COPTER_USART_ID,buff,len);
		for( i = 0; i< len; i++)
		{
		   if( mavlink_parse_char(com , buff[i], &msg, &status) ) 
		   { 
      			// Handle message
			recived = 1;
      			log("get a mavlink package; msgid = %d \n",msg.msgid);
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT: 
				{
                			break;
				}
              			default:
                			break;
      			}
    		   }
		}
	}
	return 0;
}
int do_recive_and_handle_sbus_package(int id)
{
	uint8_t buff[1024];
	int len,i,res ;

	len = do_read_usart(id , buff, 1024);
	if( len > 0 )
	{
		for( i=0; i< len; i++ )
		{
			log("%2x,",buff[i]);
		}
		log("\n");
	}
}


void check_uart_loop()
{
	int u1,u2,u3;
	u1 = do_usart_buffer_check(usart1_id, 1);
	u2 = do_usart_buffer_check(usart2_id, 1);
	u3 = do_usart_buffer_check(usart3_id, 1);
	if( u1 == 1 || u2 == 1 || u3 == 1)
	{
		while(1){
			log("uart buffer error : u1=%d,u2=%d,u3=%d\n",u1,u2,u3);
		}
	}
}

void setup()
{
	clock_setup();
	led_setup();
	usart1_id = usart1_setup(115200,8, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX_RX);
	usart2_id = usart2_setup(115200,8, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX_RX);
	usart3_id = usart3_setup(100000,8, USART_STOPBITS_2, USART_PARITY_EVEN, USART_MODE_TX);
}
void loop()
{
	static int time;
	//do_recive_and_handle_radio_message(usart2_id);
	//do_recive_and_handle_sbus_package(RADIO_USART_ID);
	log("%d s\n\r",time++);
	mdelay(1000);
}
int main(void)
{

	setup();
	while(1){
		loop();
		__asm__("nop");
	}

	while (1)
		__asm__("nop");
	return 0;
}

