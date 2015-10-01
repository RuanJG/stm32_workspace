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
#include "sbus.h"


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
int do_send_mavlink_to_uart(int id, mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int res;
	int len ;
	
	len = mavlink_msg_to_send_buffer(buf, msg);
	// Write packet via serial link
	res = do_write_usart(id, buf, len);

	return res ;	 
}



void do_change_rc_override_to_sbus(mavlink_message_t *msg)
{
	uint16_t *values;
	uint16_t num_values= 8;
	mavlink_rc_channels_override_t rc_package;
	uint8_t oframe[SBUS_FRAME_SIZE] = { 0x0f };

	mavlink_msg_rc_channels_override_decode(msg,&rc_package);
	values = &rc_package.chan1_raw;
	//px4_send_sbus_data(values,num_values);
	do_formate_rc_to_sbus(values,num_values,oframe);
	do_write_usart(SBUS_USART_ID,oframe,SBUS_FRAME_SIZE);
}

int do_recive_and_send_message(int id)
{//id = MAVLINK_COMM_0 MAVLINK_COMM_1 MAVLINK_COMM_2
	uint8_t buff[1024];
	int len,i,res, recived,com;
	mavlink_message_t msg;
	mavlink_status_t status;

	com = id -1;

	len = do_read_usart(id , buff, 1024);
	if( len > 0 ){
		//to the copter directly
		//do_write_usart(COPTER_USART_ID,buff,len);
		for( i = 0; i< len; i++)
		{
		   if( mavlink_parse_char(com , buff[i], &msg, &status) ) 
		   { 
      			// Handle message
			recived = 1;
      			log("get a mavlink package; msgid = %d \n",msg.msgid);
			if( 0 > do_send_mavlink_to_uart(COPTER_USART_ID,&msg)){
				log("Error: write the uart %d , failed\n",COPTER_USART_ID);
				return -1;
			}
			/*
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT: 
				{
                			break;
				}
              			default:
                			break;
      			}
			*/
    		   }
		}
	}
	return 0;
}
int do_recive_and_handle_sbus_package(int id)
{
	uint8_t buff[1024];
	int len,i,res, recived,com;
	mavlink_message_t msg;
	mavlink_status_t status;

	com = id -1;

	len = do_read_usart(id , buff, 1024);
	if( len > 0 ){
		//to the copter directly
		//do_write_usart(COPTER_USART_ID,buff,len);
		for( i = 0; i< len; i++)
		{
		   if( mavlink_parse_char(com , buff[i], &msg, &status) ) 
		   { 
      			// Handle message
			recived = 1;
      			log("get a mavlink package; msgid = %d \n",msg.msgid);

			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: 
				{
					do_change_rc_override_to_sbus(&msg);
                			break;
				}
              			default:
				{
					if( 0 > do_send_mavlink_to_uart(COPTER_USART_ID,&msg)){
						log("Error: write the uart %d , failed\n",COPTER_USART_ID);
						return -1;
					}
                			break;
				}
      			}
    		   }
		}
	}
	return 0;
}


// rc reciver +Telem radio => copter
void setup_one_mavlink_one_sbus()
{
	//to copter 
	usart1_id = usart1_setup(57600,8, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX_RX);
	//frome radio 
	usart2_id = usart2_setup(57600,8, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX_RX);
	//sbus out
	usart3_id = usart3_setup(100000,8, USART_STOPBITS_2, USART_PARITY_EVEN, USART_MODE_TX);
}
void loop_one_mavlink_one_sbus()
{
	do_recive_and_handle_sbus_package(RADIO_USART_ID);
}



//4G+Telem radio => copter
void setup_tow_mavlink_to_copter()
{
	//to copter 
	usart1_id = usart1_setup(57600,8, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX_RX);
	//frome radio 
	usart2_id = usart2_setup(57600,8, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX_RX);
	//4G uart
	usart3_id = usart3_setup(57600,8, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX_RX);
}
void loop_tow_mavlink_to_copter()
{
	int res;
	//first 4G uart
	res = do_recive_and_send_message(SBUS_USART_ID);
	if( res < 0 )
		while(1);
	//radio
	do_recive_and_send_message(RADIO_USART_ID);
	if( res < 0 )
		while(1);
}



void loop_check_uart()
{
	int u1,u2,u3;
	u1 = do_usart_buffer_check(usart1_id, 1);
	u2 = do_usart_buffer_check(usart2_id, 1);
	u3 = do_usart_buffer_check(usart3_id, 1);
	if( u1 == 1 || u2 == 1 || u3 == 1)
	{
		while(1){
			log("uart buffer error : u1=%d,u2=%d,u3=%d\n",u1,u2,u3);
			mdelay(1000);
		}
	}
}

void setup()
{
	clock_setup();
	led_setup();
#ifdef LOOP_TWO_MAVLINK_IN
	setup_tow_mavlink_to_copter();
#endif
#ifdef LOOP_ONE_MAVLINK_ONE_SBUS
	setup_one_mavlink_one_sbus();
#endif

}
void loop()
{
#ifdef LOOP_TWO_MAVLINK_IN
	loop_tow_mavlink_to_copter();
#endif
#ifdef LOOP_ONE_MAVLINK_ONE_SBUS
	loop_one_mavlink_one_sbus();
#endif
	loop_check_uart();
	//mdelay(1000);
}
int main(void)
{
	setup();
	while(1){
		loop();
		__asm__("nop");
	}
	return 0;
}

