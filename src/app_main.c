#include <mcu_periph/uart.h>
#include <mcu.h>
#include "mcu_periph/sys_time.h"
#include <mavlink.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <subsystems/actuators/actuators_pwm_arch.h>

#define COPTER_UART &uart1
#define TELEM_UART &uart3
#define REMOTE_4G_UART &uart2
#define SBUS_PPM_UART &uart2
#define CONSOLE_UART &uart1

//#define COPTER_COM_ID MAVLINK_COMM_0
#define REMOTE_4G_COM_ID MAVLINK_COMM_1
#define TELEM_COM_ID MAVLINK_COMM_0
//#define SBUS_PPM_COM_ID MAVLINK_COMM_1

#define DEBUG_APP 1
#define log(format, ...) if( DEBUG_APP ) printf(format, ## __VA_ARGS__)

#define CHANNELS_MAX_COUNT ACTUATORS_PWM_NB

//#include "uart.c"
int do_uart_wirte(struct uart_periph *uart,uint8_t *buff,int len)
{
	int i;
	for(i = 0; i< len; i++)
	{
		uart_put_byte(uart, buff[i]);		
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


#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))
#define SBUS_FRAME_SIZE 25

void px4_send_sbus_data(uint32_t *values, uint16_t num_values)
{
	static float sbus_scale_factor = SBUS_SCALE_FACTOR;// = ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN));
	static int  sbus_scale_offset = SBUS_SCALE_OFFSET ;//= (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f));

	uint8_t byteindex = 1; /*Data starts one byte into the sbus frame. */
	uint8_t offset = 0;
	uint16_t value;
	uint8_t oframe[SBUS_FRAME_SIZE] = { 0x0f };
	int i;

	static float last_time=0;
	float time = get_sys_time_float();
	float diff_time = time - last_time;

	diff_time = diff_time*1000;//to ms

	if (diff_time>7) {
		last_time = time;
		/* 16 is sbus number of servos/channels minus 2 single bit channels.
		* currently ignoring single bit channels.  */
		for (i = 0; (i < num_values) && (i < 16); ++i) {
	    		value = (uint16_t)(((values[i] - sbus_scale_offset) / sbus_scale_factor) + 0.5f);
	    		/*protect from out of bounds values and limit to 11 bits*/
	    		if (value > 0x07ff) {
				value = 0x07ff;
	    		}
			while (offset >= 8) {
				++byteindex;
				offset -= 8;
      			}
      			oframe[byteindex] |= (value << (offset)) & 0xff;
      			oframe[byteindex + 1] |= (value >> (8 - offset)) & 0xff;
      			oframe[byteindex + 2] |= (value >> (16 - offset)) & 0xff;
      			offset += 11;
    		}

		do_uart_wirte(SBUS_PPM_UART,oframe,SBUS_FRAME_SIZE);
		//Serial.write(oframe, SBUS_FRAME_SIZE);
		//write(sbus_fd, oframe, SBUS_FRAME_SIZE);
	}
}

void ppz_set_pwm(uint16_t *rcs, int count)
{
	int id,cnt;
	cnt = count > ACTUATORS_PWM_NB ? ACTUATORS_PWM_NB:count;
	for( id = 0; id<cnt; id ++){
		ActuatorPwmSet(id,rcs[id]);
	}
	actuators_pwm_commit();
}
void ppz_pwm_setup()
{
	int id;

	actuators_pwm_arch_init();

	for( id =0 ; id < ACTUATORS_PWM_NB ; id++ )
		ActuatorPwmSet(id,1001);
	actuators_pwm_commit();
}

void do_rc_to_pwm(uint16_t *rcs, int count)
{
	//printf("rc: %d,%d,%d,%d\n\r",rcs[0],rcs[1],rcs[2],rcs[3]);
	/*
	timer_disable_oc_output(TIM4, TIM_OC1);
	timer_set_oc_value(TIM4, TIM_OC1, rcs[0]);
	timer_enable_oc_output(TIM4, TIM_OC1);
	*/
	ppz_set_pwm(rcs,count);
}


void do_copy_uart_data_to_other_uart(struct uart_periph *uarts ,struct uart_periph *uartd)
{
	int data_len,i;
	uint8_t buff[UART_RX_BUFFER_SIZE];

	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++)
			buff[i] = uart_getch(uarts);
	
		for( i = 0 ; i< data_len; i++)
			uart_put_byte(uartd, buff[i] );
	}
}
void do_copy_uart_data_to_two_uart( struct uart_periph *uarts ,struct uart_periph *uartd1, struct uart_periph *uartd2)
{
	int data_len,i;
	uint8_t c;
	uint8_t buff[UART_RX_BUFFER_SIZE];
	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++){
			buff[i] = uart_getch(uarts);
		}
		for( i = 0 ; i< data_len; i++){
			uart_put_byte(uartd1, buff[i] );
		}
		for( i = 0 ; i< data_len; i++){
			uart_put_byte(uartd2, buff[i] );
		}
	}
}
void do_copy_uart_and_handle_mavlink_msg(struct uart_periph *uarts ,struct uart_periph *uartd)
{
	mavlink_message_t msg; 
	mavlink_status_t status;
	int data_len,i;
	uint8_t buff[UART_RX_BUFFER_SIZE];
	uint16_t rcs[8];

	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++)
			buff[i] = uart_getch(uarts);
	
		if( uartd != NULL)
			for( i = 0 ; i< data_len; i++)
				uart_put_byte(uartd, buff[i] );
	
		for( i = 0 ; i< data_len; i++){
			if( mavlink_parse_char(TELEM_COM_ID, buff[i], &msg, &status) ) { 
      				switch(msg.msgid) {
					case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
						rcs[0] = mavlink_msg_rc_channels_override_get_chan1_raw(&msg);
						rcs[1] = mavlink_msg_rc_channels_override_get_chan2_raw(&msg);
						rcs[2] = mavlink_msg_rc_channels_override_get_chan3_raw(&msg);
						rcs[3] = mavlink_msg_rc_channels_override_get_chan4_raw(&msg);
						rcs[4] = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);
						rcs[5] = mavlink_msg_rc_channels_override_get_chan6_raw(&msg);
						rcs[6] = mavlink_msg_rc_channels_override_get_chan7_raw(&msg);
						rcs[7] = mavlink_msg_rc_channels_override_get_chan8_raw(&msg);
						do_rc_to_pwm(rcs,CHANNELS_MAX_COUNT);

					}
					default:
       		         			break;
      				}
    			}
		}
  	}
}
void do_sbus_send_pwm(int32_t * rcs, int count)
{
	px4_send_sbus_data(rcs,count);
}

void loop_4g_and_telem_to_copter()
{
	do_copy_uart_data_to_two_uart(COPTER_UART,REMOTE_4G_UART,TELEM_UART);
	//do_copy_uart_data_to_other_uart(REMOTE_4G_UART,COPTER_UART);
	do_copy_uart_data_to_other_uart(TELEM_UART,COPTER_UART);
	//do_copy_uart_and_handle_mavlink_msg(TELEM_UART,COPTER_UART);
}
void loop_telem_and_sbusppm_to_copter()
{
	do_copy_uart_data_to_other_uart(COPTER_UART,TELEM_UART);
	do_copy_uart_and_handle_mavlink_msg(TELEM_UART,COPTER_UART);
	do_sbus_send_pwm(actuators_pwm_values,CHANNELS_MAX_COUNT);
	;//mavlink_parse_char
}
void loop_telem_to_pwm()
{
	do_copy_uart_and_handle_mavlink_msg(TELEM_UART,NULL);
}
void setup()
{
	mcu_init(); //define PERIPHERALS_AUTO_INIT in makefile , and will init preiph auto in mcu_init(), like uartx_init()
	//pwm_timer_setup();
	ppz_pwm_setup();
	//uart define config in makefile and uart_arch.c 
  	//uart_periph_set_mode(&uart1, USE_UART1_TX, USE_UART1_RX, UART1_HW_FLOW_CONTROL);
  	//uart_periph_set_bits_stop_parity(&uart1, UART1_BITS, UART1_STOP, UART1_PARITY);
  	//uart_periph_set_baudrate(&uart1, UART1_BAUD);
}
void loop()
{
	//loop_4g_and_telem_to_copter();
	//loop_telem_and_sbusppm_to_copter();
	loop_telem_to_pwm();
	//delay_ms(10);
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

