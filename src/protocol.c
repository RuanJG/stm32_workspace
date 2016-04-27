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
#include "crc_sbus.h"
#include "mini_sbus.h"
#include "protocol.h"





#define PROTOCOL_USE_SBUS 0
#define PROTOCOL_USE_CRC_SBUS 0
#define PROTOCOL_USE_MINI_SBUS 1


#if PROTOCOL_USE_CRC_SBUS
#define protocol_parse parse_crc_sbus_frame
#define protocol_send send_crc_sbus_out

#elif PROTOCOL_USE_SBUS
#define protocol_parse parse_sbus_frame
#define protocol_send send_sbus_out

#elif PROTOCOL_USE_MINI_SBUS
#define protocol_parse parse_mini_sbus_frame
#define protocol_send send_mini_sbus_out
#else 
// error 
#endif


#define log(format, ...) printf(format, ## __VA_ARGS__)

struct sbus_buffer sbus_buf;
uint8_t oframe[SBUS_FRAME_SIZE];
uint8_t crc_oframe[CRC_SBUS_FRAME_SIZE];
int sbus_t;

void protocol_send_func(uint8_t* data, int len)
{
	int i;
	uint8_t flag;
	for(i=0; i< len; i++){
		protocol_parse(data[i],&sbus_buf);
	}
}

void test_protocol_setup()
{
	sbus_buffer_init(&sbus_buf);
	sbus_t =sys_time_register_timer(0.001,NULL);//10ms 
}
void test_protocol_loop()
{
	uint16_t rc_chans[16]={1000} ;
	int i;
	volatile int value;
	volatile int ret;
	int j;
	volatile int count = 0;
	uint8_t crc_oframe[CRC_SBUS_FRAME_SIZE] = { 0x0f };

	sbus_buffer_init(&sbus_buf);
	for( value = 900; value < 0x800; value++){
		for( i=0 ;i < 16; i++ ) rc_chans[i] = value;

#if 0
	encode_crc_sbus_frame(rc_chans, 16, crc_oframe);
     //         decode_sbus_frame(crc_oframe, sbus_buf.rc_chans, &sbus_buf.frame_available);
     	test_crc_sbus_send_func(crc_oframe, CRC_SBUS_FRAME_SIZE);
#else
		protocol_send(rc_chans,16,&protocol_send_func);
#endif
		log("o=%d\r\n",rc_chans[0]);
		for( i=0 ;i < 16; i++ ){
			log("%d,",sbus_buf.rc_chans[i]);
			if( rc_chans[i] != sbus_buf.rc_chans[i] ){
				count++;
				break;
			}
		}
		log("\r\n");
		sys_time_usleep(10000);
	}
	log("test over , failse times %d in %d\r\n",count,0x800-500);
	log("sbus_buffer: fram_err=%d , decode_err=%d\r\n",sbus_buf.frame_capture_faile,sbus_buf.frame_decode_faile);
	count = 0;
	sys_time_usleep(1000000);
}




