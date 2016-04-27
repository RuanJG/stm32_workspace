#include <mcu_periph/uart.h>
#include <mcu.h>
#include "mcu_periph/sys_time.h"
#include <mavlink.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <subsystems/actuators/actuators_pwm_arch.h>

#include "sbus.h"
#include "pwm.h"
#include "crc_sbus.h"
#include "mini_sbus.h"

#define CONSOLE_UART &uart1

#define DEBUG_APP 1
#define log(format, ...) if( DEBUG_APP ) printf(format, ## __VA_ARGS__)

uint8_t signel_lost = 0;
float last_recive_data_time = 0;

//#include "uart.c"
inline int do_uart_wirte(struct uart_periph *uart,uint8_t *buff,int len)
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

inline void sys_time_msleep(int ms)
{
	sys_time_usleep(ms*1000);		
}


/*
*
*  ####################### signal 
*/
int signel_t;
#define SIGNEL_LOST_S 1.0
void check_signel_task()
{
	float new_time;
	if( signel_lost == 0 ){
		new_time = get_sys_time_float();
		if( (new_time - last_recive_data_time) > SIGNEL_LOST_S ){
			signel_lost = 1;
			pwm_stop();
		}
	}
}
inline void call_signel_connected()
{
	if( signel_lost ){
		signel_lost = 0;
		pwm_start();
	}
	last_recive_data_time = get_sys_time_float();
}
void check_signel_setup()
{
	signel_lost = 1;
	signel_t = sys_time_register_timer(SIGNEL_LOST_S/2, NULL);
}
inline void check_signel_loop()
{
	if( sys_time_check_and_ack_timer(signel_t) ){
		check_signel_task();
	}
}




#ifdef SBUS_OUT_UART
int sbus_out_t;
static uint8_t sbus_out_oframe[SBUS_FRAME_SIZE] = { 0x0f };
void sbus_out_setup()
{
	sbus_out_t = sys_time_register_timer(0.007, NULL);
}
inline void sbus_out_loop()
{
	if (signel_lost!=1 && sys_time_check_and_ack_timer(sbus_out_t)) {
#if SBUS_OUT_UART_USE_RECIVE_BUFF
		//update oframe in crc_sbus_in_uart_loop
#else
		encode_sbus_frame(actuators_pwm_values , ACTUATORS_PWM_NB , sbus_out_oframe);
#endif
		do_uart_wirte(&(SBUS_OUT_UART),sbus_out_oframe, SBUS_FRAME_SIZE);
	}
}
#endif

#ifdef CRC_SBUS_OUT_UART
//struct sbus_buffer sbus_out;
int crc_sbus_out_t;
static uint8_t crc_sbus_out_oframe[CRC_SBUS_FRAME_SIZE] = { 0x0f };
void crc_sbus_out_setup()
{
	crc_sbus_out_t = sys_time_register_timer(0.005, NULL);
}
inline void crc_sbus_out_loop()
{
	if (sys_time_check_and_ack_timer(crc_sbus_out_t)) {
		encode_crc_sbus_frame(actuators_pwm_values , ACTUATORS_PWM_NB , crc_sbus_out_oframe);
		do_uart_wirte(&(CRC_SBUS_OUT_UART),crc_sbus_out_oframe, CRC_SBUS_FRAME_SIZE);
	}
}
#endif


/*
*  ###################################  crc_sbus 0 and 1 channel
*
*/
#ifdef CRC_SBUS_0_IN_UART
struct sbus_buffer crc_sbus0_uart_buf;
#endif
#ifdef CRC_SBUS_1_IN_UART
struct sbus_buffer crc_sbus1_uart_buf;
#endif
#if CRC_SBUS_IN_DEBUG
int crc_sbus_debug_t = -1;
#endif
void crc_sbus_in_uart_setup(struct sbus_buffer *sbus_b)
{
	sbus_buffer_init(sbus_b);
#if CRC_SBUS_IN_DEBUG
	if( crc_sbus_debug_t < 0 )
		crc_sbus_debug_t = sys_time_register_timer(1.0, NULL);
#endif
}
inline void crc_sbus_in_uart_loop(struct uart_periph *uart , struct sbus_buffer *sbus_b)
{
	int data_len;
	int i,ret;
	data_len = uart_char_available(uart);
	if( data_len > 0 ){
#if USE_CRC_MINI_SBUS_IN
		ret = 0;
		for( i=0; i< data_len ; i++){
			if( 1==  parse_mini_sbus_frame(uart_getch(uart), sbus_b ) ){
				if( sbus_b->frame_available == TRUE ){
					pwm_recovery_part_rcs(sbus_b->rc_chans, 8 , (0x0f&sbus_b->flag));
					sbus_b->frame_available = FALSE;
					ret = 1;
			#ifdef SBUS_OUT_UART
				#if SBUS_OUT_UART_USE_RECIVE_BUFF
					if( (sbus_b->flag & 0x0f) == 0){
						memcpy(sbus_out_oframe,sbus_b->buffer,SBUS_FRAME_SIZE);
						sbus_out_oframe[SBUS_END_BYTE_IDX] = SBUS_END_BYTE;
					}
				#endif
			#endif
				}
			}
		}
		if( ret ){
			call_signel_connected();
		}
#else
		for( i=0; i< data_len ; i++){
			if( 1==  parse_crc_sbus_frame(uart_getch(uart), sbus_b ) ){
			#ifdef SBUS_OUT_UART
				#if SBUS_OUT_UART_USE_RECIVE_BUFF
				memcpy(sbus_out_oframe,sbus_b->buffer,SBUS_FRAME_SIZE);
				sbus_out_oframe[SBUS_END_BYTE_IDX] = SBUS_END_BYTE;
				#endif
			#endif
			}
		}
		if( sbus_b->frame_available == TRUE ){
			pwm_recovery_rcs(sbus_b->rc_chans , SBUS_MAX_RC_COUNT);
			sbus_b->frame_available = FALSE;
			call_signel_connected();
		}
#endif
	}
}
#if CRC_SBUS_IN_DEBUG
inline void crc_sbus_in_uart_debug()
{
	if( sys_time_check_and_ack_timer(crc_sbus_debug_t)) {
#ifdef CRC_SBUS_0_IN_UART
		if( crc_sbus0_uart_buf.frame_count>0)
			crc_sbus0_uart_buf.rssi = (float)(crc_sbus0_uart_buf.frame_count-crc_sbus0_uart_buf.frame_decode_faile-crc_sbus0_uart_buf.frame_capture_faile)/crc_sbus0_uart_buf.frame_count ;
		else
			crc_sbus0_uart_buf.rssi = 0.0;
		log("crc_sbus0 : decode error:%d, frame error:%d , all %d, %f\r\n", \
			crc_sbus0_uart_buf.frame_decode_faile, \
			crc_sbus0_uart_buf.frame_capture_faile, \
			crc_sbus0_uart_buf.frame_count, \
			crc_sbus0_uart_buf.rssi);
		crc_sbus0_uart_buf.frame_decode_faile = 0;
		crc_sbus0_uart_buf.frame_capture_faile = 0;
		crc_sbus0_uart_buf.frame_count = 0;
	#if 1
		for(int j=0; j< SBUS_MAX_RC_COUNT; j++) log("%d,",crc_sbus0_uart_buf.rc_chans[j]);
		log("\r\n");
	#endif
#endif
#ifdef CRC_SBUS_1_IN_UART
		if( crc_sbus1_uart_buf.frame_count>0)
			crc_sbus1_uart_buf.rssi = (crc_sbus1_uart_buf.frame_count-crc_sbus1_uart_buf.frame_decode_faile-crc_sbus1_uart_buf.frame_capture_faile)/crc_sbus1_uart_buf.frame_count ;
		else
			crc_sbus1_uart_buf.rssi = 0.0;
		log("crc_sbus1 : decode error:%d, frame error:%d , all %d, %f\r\n", \
			crc_sbus1_uart_buf.frame_decode_faile, \
			crc_sbus1_uart_buf.frame_capture_faile, \
			crc_sbus1_uart_buf.frame_count, \
			crc_sbus1_uart_buf.rssi);
		crc_sbus1_uart_buf.frame_decode_faile = 0;
		crc_sbus1_uart_buf.frame_capture_faile = 0;
		crc_sbus1_uart_buf.frame_count = 0;
	#if 1
		for(int j=0; j< SBUS_MAX_RC_COUNT; j++) log("%d,",crc_sbus1_uart_buf.rc_chans[j]);
		log("\r\n");
	#endif
#endif
	}
}
#endif


#if USE_FAKE_RC_IN
int fake_rc_in_t ;
void fake_rc_in_setup()
{
	fake_rc_in_t = sys_time_register_timer(0.004, NULL);
}

void fake_rc_in_loop(){
	int i;
	static uint16_t rc=1000;
	if( sys_time_check_and_ack_timer(fake_rc_in_t) ){
		rc += 1;
		if( rc > 2000 ) rc = 1000;
		for( i=0 ; i< ACTUATORS_PWM_NB; i++){
			ActuatorPwmSet(i,rc);
		}
		call_signel_connected();
	}
}
#endif

void setup()
{
	mcu_arch_init();
	sys_time_init();
#if USE_UART0
 	uart0_init();
#endif
#if USE_UART1
	uart1_init();
#endif
#if USE_UART2
	uart2_init();
#endif
#if USE_UART3
	uart3_init();
#endif
	check_signel_setup();
#ifdef SBUS_OUT_UART
	sbus_out_setup();
#endif
#ifdef CRC_SBUS_OUT_UART
	crc_sbus_out_setup();
#endif
#ifdef CRC_SBUS_0_IN_UART
	crc_sbus_in_uart_setup(&crc_sbus0_uart_buf);
#endif
#ifdef CRC_SBUS_1_IN_UART
	crc_sbus_in_uart_setup(&crc_sbus1_uart_buf);
#endif
	pwm_setup();

#if USE_FAKE_RC_IN
	fake_rc_in_setup();
#endif

#if 0
	test_protocol_setup();
#endif

}
inline void loop()
{
#if 0
	test_protocol_loop();
#endif
#if USE_FAKE_RC_IN
	fake_rc_in_loop();
#endif
#ifdef CRC_SBUS_0_IN_UART
	crc_sbus_in_uart_loop(&(CRC_SBUS_0_IN_UART), &crc_sbus0_uart_buf);
#endif
#ifdef CRC_SBUS_1_IN_UART
	crc_sbus_in_uart_loop(&(CRC_SBUS_1_IN_UART), &crc_sbus1_uart_buf);
#endif

	check_signel_loop();

#ifdef SBUS_OUT_UART
	sbus_out_loop();
#endif
#ifdef CRC_SBUS_OUT_UART
	crc_sbus_out_loop();
#endif
#if CRC_SBUS_IN_DEBUG
	crc_sbus_in_uart_debug();
#endif
}

int main(void)
{
	setup();
	while(1){
		loop();
	}
	return 0;
}

