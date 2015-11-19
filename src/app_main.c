#include <mcu_periph/uart.h>
#include <mcu_periph/adc.h>
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
		ActuatorPwmSet(id,1200);
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


#define NB_ADC 6
#define ADC_NB_SAMPLES 16
static struct adc_buf buf_adc[NB_ADC];

#define MAX_RC_COUNT 6
#define ZFRAME_RCS_INT_COUNT 2 // MAX_RC_COUNT/3; as each int store 3 rc, (MAX_RC_COUNT/3)=2=int[2] ; (tag(16bit)+ 2*32bit+ crc(16bit)) = 10*8bit 
#define ZFRAME_BUFF_SIZE 13 //tag(2) + rcs(4*ZFRAME_RCS_INT_COUNT) + crc(2) + len(1) = 12 
#define ZFRAME_TAG 0x3ff
#define is_zframe_tag(L,H) (L == 0xff && H == 0x03)
#define zframe_get_tag(buff) (buff[0]|(buff[1]<<8))
#define zframe_get_count(buff) (buff[2])
#define zframe_get_rcs_point(buff) ( (uint32_t*)&buff[3])
#define zframe_get_crc(buff) ( buff[11]| (buff[12]<<8) )
typedef struct _zframe_packet{
	uint16_t tag;
	uint8_t count;
	uint32_t rcs[ZFRAME_RCS_INT_COUNT];//MAX_RC_COUNT/3 , rcs[0]=rc1-0-rc2-0-rc3 ....
	uint16_t crc; //must put in the end
}zframe_packet_t;

//static uint16_t rcs[MAX_RC_COUNT];
inline uint16_t _get_rc(int id)
{
	return buf_adc[id].sum / ADC_NB_SAMPLES;
}
#define MAX_RC_VALUE 2024
#define MIN_RC_VALUE 1000
#define scale_rc_0_to_2048(rc) (rc>>1)
#define scale_rc_0_to_1024(rc) (rc>>2)

#define get_rc_for_mavlink(id) ( MIN_RC_VALUE + scale_rc_0_to_1024(_get_rc(id)))
inline uint16_t get_rc_for_reciver(id) {
	uint16_t rc = scale_rc_0_to_1024(_get_rc(id)) ;
	if( rc >= 0x3ff ) rc = 0x3fe; //so the max data is 0x3fe,and the tag is 0x3ff
	return rc;
}

void do_dead(char *last_string)
{
	log("system Dead: %s\n\r",last_string);
	while(1) 
		delay_ms(60000);
}

int check_zframe_packet(uint8_t* packet)
{
	uint16_t crc,crc_ori;
	uint16_t tag;
	
	tag = zframe_get_tag(packet);
	if( tag != 0x3ff){
		log("packet.tag error =%02x\n\r",tag);
		return -1;
	}
	if( zframe_get_count(packet) != MAX_RC_COUNT ){
		log("packet.count error =%d\n\r",zframe_get_count(packet));
		return -1;
	}
	crc_ori = zframe_get_crc(packet);
	crc = crc_calculate( packet, ZFRAME_BUFF_SIZE-2);
	if( crc != crc_ori ){
		log("packet.crc error,ori =%02x, cail=%02x \n\r",crc_ori, crc);
		return -1;
	}
	return 0;

}
int decode_zframe(uint8_t* packet, uint16_t *rd)
{
	int error,i,bit_cnt,rc_index;
	uint32_t * p32;

	error = check_zframe_packet(packet);
	if( error != 0 ){
		log("packet error\n\r");
		return -1;
	}

	p32 = zframe_get_rcs_point(packet);
	bit_cnt = 0;
	rc_index = 0;
	for( i=0; i< MAX_RC_COUNT && rc_index <ZFRAME_RCS_INT_COUNT ; i++){
		rd[i]= (p32[rc_index]>>bit_cnt) & 0x3ff;	
		bit_cnt+=11;
		if( bit_cnt > 22 ){
			bit_cnt = 0;
			rc_index++;
		}
	}
	if( i < MAX_RC_COUNT ){
		log("decode_zframe rc count is not right\n\r");
		return -1;
	}
	return 0;
}


void handle_zframe_packet(uint8_t * data)
{
	int error,i;
	uint16_t rc_data[MAX_RC_COUNT];

	error = decode_zframe(data,rc_data);

	log("get rc:\n\r");
	if( error == 0 ){
		for(i=0 ; i< MAX_RC_COUNT; i++) 
			log("RC%d=%d,",i,rc_data[i]);
	}else{
		do_dead("decode_zframe error");
	}
	log("...\n\r");
}
void collect_zframe(uint8_t *data, int len)
{
	static uint8_t recive_buff[ZFRAME_BUFF_SIZE]={0};
	static int index0 = 0;

	int i;
	int packet_start = 0;

	//log("collect_zframe: start \n\r");
	for( i = 0; i< len; i++){
		if( index0 >= ZFRAME_BUFF_SIZE){
/*
	for( int j=0 ; j< len; j++) log("%02x,",recive_buff[j]);
	log("\n\r");
	*/
			handle_zframe_packet(recive_buff);
			index0 = 0;
		}
		if( index0 == 0 ){
			if( is_zframe_tag(data[0],data[1]) ){
				recive_buff[index0++]=data[i++];
				recive_buff[index0++]=data[i];
			}else{
				log("ignoring data");
			}
		}else{
			recive_buff[index0++]=data[i];
		}
	}
	//log("collect_zframe: end\n\r");
}




void encode_rcs(uint8_t *buff)
{
	int id,index;
	int count;
	uint32_t *p32;
	uint16_t crc;

	//tag
	buff[0] = 0xff;buff[1] = 0x03;
	//count
	buff[2] = MAX_RC_COUNT;
	//rc
	p32 = (uint32_t *)&buff[3];
	memset(p32,0, sizeof(uint32_t)*ZFRAME_RCS_INT_COUNT);
	for( id=0,index=0,count=0; id < MAX_RC_COUNT; id++,count+=11){
		if( count > 22 ){//count = 0 11 22
			count = 0; 
			index++;
		}
		p32[index] |= ( get_rc_for_reciver(id) << count );//10bit
	}
	//crc
	crc = crc_calculate( buff, ZFRAME_BUFF_SIZE-2);
	buff[ZFRAME_BUFF_SIZE-2]=crc&0xff;
	buff[ZFRAME_BUFF_SIZE-1]=(crc>>8)&0xff;
}

void send_data_to_emitter(uint8_t* data, int len)
{
	/*log("send_data_to_emitter:\n\r");
	for( int i=0 ; i< len; i++) log("%02x,",data[i]);
	log("\n\r");
	*/
	collect_zframe(data,len);
}

void send_rc_to_reciver()
{
	uint8_t buff[ZFRAME_BUFF_SIZE];

	encode_rcs(buff);
	send_data_to_emitter(buff,ZFRAME_BUFF_SIZE); 
}



inline void setup()
{
	//mcu_init(); //define PERIPHERALS_AUTO_INIT in makefile , and will init preiph auto in mcu_init(), like uartx_init()
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

	adc_init();
#ifdef ADC_1
  adc_buf_channel(ADC_1, &buf_adc[0], ADC_NB_SAMPLES);
#endif
#ifdef ADC_2
  adc_buf_channel(ADC_2, &buf_adc[1], ADC_NB_SAMPLES);
#endif
#ifdef ADC_3
  adc_buf_channel(ADC_3, &buf_adc[2], ADC_NB_SAMPLES);
#endif
#ifdef ADC_4
  adc_buf_channel(ADC_4, &buf_adc[3], ADC_NB_SAMPLES);
#endif
#ifdef ADC_5
  adc_buf_channel(ADC_5, &buf_adc[4], ADC_NB_SAMPLES);
#endif
#ifdef ADC_6
  adc_buf_channel(ADC_6, &buf_adc[5], ADC_NB_SAMPLES);
#endif

mcu_int_enable();


	//ppz_pwm_setup();
}

inline void update_adc_value()
{
  uint16_t values[NB_ADC];
  uint8_t i;
  for (i = 0; i < NB_ADC; i++) {
    values[i] = buf_adc[i].sum / ADC_NB_SAMPLES;
    log("adc%d=%d,",i,values[i]);
  }
  log("\n\r");
}
inline void loop()
{
	//loop_4g_and_telem_to_copter();
	//loop_telem_and_sbusppm_to_copter();
	
	delay_ms(100);
	log("1s\n\r");
	//update_adc_value();
	send_rc_to_reciver();
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

