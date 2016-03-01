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









//******************************************************  curve expo

#define RESX_SHIFT 10
#define RESX       1024
#define RESXu      1024u
#define RESXul     1024ul
#define RESXl      1024l

#define CORRECT_NEGATIVE_SHIFTS
int16_t calc100to256_16Bits(int16_t x) // return x*2.56
{
    // y = 2*x + x/2 +x/16-x/512-x/2048
    // 512 and 2048 are out of scope from int8 input --> forget it
#ifdef CORRECT_NEGATIVE_SHIFTS
    int16_t res=(int16_t)x<<1;
    //int8_t  sign=(uint8_t) x>>7;
    int8_t sign=(x<0?1:0);
    x-=sign;
    res+=(x>>1);
    res+=sign;
    res+=(x>>4);
    res+=sign;
    return res;
#else
    return ((int16_t)x<<1)+(x>>1)+(x>>4);
#endif
}
int16_t calc100to256(int8_t x) // return x*2.56
{
  return calc100to256_16Bits(x);
}
// expo-funktion:
// ---------------
// kmplot
// f(x,k)=exp(ln(x)*k/10) ;P[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
// f(x,k)=x*x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
// f(x,k)=x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
// f(x,k)=1+(x-1)*(x-1)*(x-1)*k/10 + (x-1)*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
// don't know what this above should be, just confusing in my opinion,

// here is the real explanation
// actually the real formula is
/*
 f(x) = exp( ln(x) * 10^k)
 if it is 10^k or e^k or 2^k etc. just defines the max distortion of the expo curve; I think 10 is useful
 this gives values from 0 to 1 for x and output; k must be between -1 and +1
 we do not like to calculate with floating point. Therefore we rescale for x from 0 to 1024 and for k from -100 to +100
 f(x) = 1024 * ( e^( ln(x/1024) * 10^(k/100) ) )
 This would be really hard to be calculated by such a microcontroller
 Therefore Thomas Husterer compared a few usual function something like x^3, x^4*something, which look similar
 Actually the formula
 f(x) = k*x^3+x*(1-k)
 gives a similar form and should have even advantages compared to a original exp curve.
 This function again expect x from 0 to 1 and k only from 0 to 1
 Therefore rescaling is needed like before:
 f(x) = 1024* ((k/100)*(x/1024)^3 + (x/1024)*(100-k)/100)
 some mathematical tricks
 f(x) = (k*x*x*x/(1024*1024) + x*(100-k)) / 100
 for better rounding results we add the 50
 f(x) = (k*x*x*x/(1024*1024) + x*(100-k) + 50) / 100

 because we now understand the formula, we can optimize it further
 --> calc100to256(k) --> eliminates /100 by replacing with /256 which is just a simple shift right 8
 k is now between 0 and 256
 f(x) = (k*x*x*x/(1024*1024) + x*(256-k) + 128) / 256
 */

// input parameters;
//  x 0 to 1024;
//  k 0 to 100;
// output between 0 and 1024
// #define EXTENDED_EXPO
// increases range of expo curve but costs about 82 bytes flash
unsigned int expou(unsigned int x, unsigned int k)
{
#if defined(EXTENDED_EXPO)
  bool extended;
  if (k>80) {
    extended=true;
  }
  else {
    k += (k>>2);  // use bigger values before extend, because the effect is anyway very very low
    extended=false;
  }
#endif

  k = calc100to256(k);

  uint32_t value = (uint32_t) x*x;
  value *= (uint32_t)k;
  value >>= 8;
  value *= (uint32_t)x;

#if defined(EXTENDED_EXPO)
  if (extended) {  // for higher values do more multiplications to get a stronger expo curve
    value >>= 16;
    value *= (uint32_t)x;
    value >>= 4;
    value *= (uint32_t)x;
  }
#endif

  value >>= 12;
  value += (uint32_t)(256-k)*x+128;

  return value>>8;
}

int expo(int x, int k)
{
  if (k == 0) return x;
  int y;
  int neg = (x < 0)?1:0;

  if (neg==1) x = -x;
  if (k<0) {
    y = RESXu-expou(RESXu-x, -k);
  }
  else {
    y = expou(x, k);
  }
  return neg==1? -y : y;
}
//**************************************************************** curve expo





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

void delay_ms(float ms)
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

void do_rc_config( mavlink_message_t * msg);

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
	
		for( i = 0 ; i< data_len && uartd != NULL; i++)
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
						break;
					}
					case MAVLINK_MSG_ID_RC_CHANNELS: {
						do_rc_config(&msg);
						break;
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

#define ADC_MIN_ID 0
#define ADC_MIDDLE_ID 1
#define ADC_MAX_ID 2
#define ADC_RANGE_ID 3
uint16_t adc_sensor_range[NB_ADC][4]={
/* fushi jostick param
	{1020,2281,3460, 2440},
	{710, 1967, 3200, 2390},
	{800, 1949, 3150, 2350},
	{640, 1829, 3150, 2530},
	{0,2048,4095,4095},
	{0,2048,4095,4095}
*/
///* sumsang jostick param
	//the [3] is range , in adc_sensor_cali_init will fill it 
	{100,2044,3800,0},
	{200, 2043, 3700,0},
	{300, 2030, 3750,0},
	{400, 2160, 4090,0},
	{0,2045,4095,0},
	{0,2045,4095,0}
//*/
};
float adc_sensor_again[NB_ADC][2]={
	// adc_sensor_range[1] - adc_sensor_range[0] / 500
	{0.0,0.0},	
	{0.0,0.0},	
	{0.0,0.0},	
	{0.0,0.0},	
	{0.0,0.0},	
	{0.0,0.0}	
};
void adc_sensor_cali_init()
{
	int i;
	for( i=0; i< NB_ADC; i++){
		/*
		if( adc_sensor_range[i][1] > 0 ){
			adc_sensor_again[i][0] = 500.0/(adc_sensor_range[i][1]-adc_sensor_range[i][0]);
			adc_sensor_again[i][1] = 500.0/(adc_sensor_range[i][2]-adc_sensor_range[i][1]);
		}else{
			adc_sensor_again[i][0] = 0.0;
			adc_sensor_again[i][1] = 0.0;
		}
		log("%f,  %f \r\n",adc_sensor_again[i][0],adc_sensor_again[i][1]);
		*/
		adc_sensor_range[i][ADC_RANGE_ID] = adc_sensor_range[i][ADC_MAX_ID] - adc_sensor_range[i][ADC_MIN_ID];
	}
}

#define RC_MAX_COUNT NB_ADC
#define RC_RANGE_VALUE 1000 //1022 //1000~ 1000+1023 , this may effect adc_sensor_cali_init
#define RC_MIN_VALUE 1000
#define RC_MAX_VALUE 2000 //2022 // 
#define RC_ROLL_ID 0
#define RC_PITCH_ID 1
#define RC_THR_ID 2
#define RC_YAW_ID 3
#define RC_AUX1_ID 4
#define RC_AUX2_ID 5
uint32_t rc_revert_mask = 0;//|1<<RC_PITCH_ID|1<<RC_ROLL_ID; // 
uint16_t rc_user_range[RC_MAX_COUNT][2]=
{// if any vale==0, no userful
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0}
};
inline uint16_t get_rc_min_value(int id){
	if( rc_user_range[id][0] != 0 )
		return rc_user_range[id][0];
	return RC_MIN_VALUE;
}
inline uint16_t get_rc_max_value(int id){
	if( rc_user_range[id][1] != 0 )
		return rc_user_range[id][1];
	return RC_MAX_VALUE;
}

#define CURVE_THR_TYPE 1
#define CURVE_MIDDLE_TYPE 2
short rc_user_cuver[RC_MAX_COUNT][2]={
	//[paramk[-100~100],type[1 thr,2 middle]
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0}
};
short rc_user_trim[RC_MAX_COUNT]={0};//+- 100
uint16_t rc_value_list[RC_MAX_COUNT]={0};
typedef struct _rc_mix_type {
	char main_id ;
	char slave_id ;
	float add_persen ;
	float sub_persen ;
	char start_position ;
	char valiable ;
}rc_mix_type;
#define MIX_LIST_MAX_COUNT 4
rc_mix_type mix_list[MIX_LIST_MAX_COUNT];
void init_rc_mix_list()
{
	int i;
	for( i=0 ; i< MIX_LIST_MAX_COUNT; i++)
		mix_list[i].valiable = 0;
}
rc_mix_type *get_empty_rc_mix()
{
	int i;
	for ( i=0; i< MIX_LIST_MAX_COUNT; i++){
		if( mix_list[i].valiable == 0 )
			return &mix_list[i];
	}
	return NULL;
}
#define BASE_CONFIG_ID 0
#define MIX_CONFIG_ID 1
void do_rc_config( mavlink_message_t * msg)
{
	mavlink_rc_channels_t packet;
	int id,curve_type,min,max,trim,revert;
	short curve_paramk;
	short tmp;
	rc_mix_type *rc_mix;


	mavlink_msg_rc_channels_decode(msg,&packet);

	if( packet.time_boot_ms == BASE_CONFIG_ID){
		id = packet.chan1_raw;
		if( id < 0 || id >= RC_MAX_COUNT){
			log("do_rc_config: id is no fix\r\n");
			return;
		}
		log("do_rc_config: id=%d\r\n",id);
		//-----------  curve 
		curve_paramk = packet.chan2_raw;
		curve_type = packet.chan3_raw;
		//log("get curve_paramk=%d\r\n",curve_paramk);
		if( curve_paramk >= -100 && curve_paramk <= 100 ){
			log("set curve_paramk=%d\r\n",curve_paramk);
			rc_user_cuver[id][0] = curve_paramk;
		}
		if( curve_type == CURVE_THR_TYPE || curve_type == CURVE_MIDDLE_TYPE ){
			log("set curve_type=%d\r\n",curve_type);
			rc_user_cuver[id][1] = curve_type;
		}
		//-------------- MIN MAX
		min = packet.chan5_raw;
		max = packet.chan4_raw;
		if( min > max ){
			min = max; 
			max = packet.chan5_raw;
		}
		if( max > RC_MAX_VALUE ) max = RC_MAX_VALUE;
		if( min != 0 && max != 0 ){
			rc_user_range[id][0]=min;
			rc_user_range[id][1] = max;
			log("set min=%d,max=%d\r\n",min,max);
		}
		//-------------- trim
		trim = packet.chan6_raw;
		if( trim >= -200 && trim <= 200 ){
			rc_user_trim[id] = trim;
			log("set trim=%d\r\n",trim);
		}
		//--------------- revert
		revert = packet.chan7_raw;
		if( revert == 1 ){
			rc_revert_mask |= 1<<id;
			log("set revert=%d\r\n",revert);
		}else{
			rc_revert_mask &= ~(1<<id);
			log("set revert=%d\r\n",revert);
		}
	}else if(packet.time_boot_ms == MIX_CONFIG_ID){
		rc_mix = get_empty_rc_mix();
		if( rc_mix == NULL ) {
			log("no empty mix for new one\r\n");
			return;
		}
		//main id
		tmp = packet.chan1_raw;
		if( tmp < 0 || tmp >= RC_MAX_COUNT){
			log("bad mix maind id\r\n");
			return;
		}
		rc_mix->main_id = tmp;
		log("mix main id=%d\r\n",tmp);
		//slave_id
		tmp = packet.chan2_raw;
		if( tmp < 0 || tmp >= RC_MAX_COUNT ){
			log("bad mix slave id\r\n");
			return;
		}
		rc_mix->slave_id = tmp;
		log("mix slave id=%d\r\n",tmp);
		//start position
		tmp = packet.chan3_raw;
		rc_mix->start_position = tmp;
		log("mix start position=%d\r\n",tmp);
		//add_persen
		tmp = packet.chan4_raw;
		if( tmp < -100 || tmp > 100 ){
			log("bad mix add persen=%d\r\n",tmp);
			return;
		}
		rc_mix->add_persen = tmp/100;
		log("mix add persen=%f\r\n",rc_mix->add_persen);
		//sub_persen
		tmp = packet.chan5_raw;
		if( tmp < -100 || tmp > 100 ){
			log("bad mix sub persen=%d\r\n",tmp);
			return;
		}
		rc_mix->sub_persen = tmp/100;
		log("mix sub persen=%f\r\n",rc_mix->sub_persen);

		rc_mix->valiable = 1;
	}
}
inline uint16_t get_adc(int id)
{
	//return id<NB_ADC ? (buf_adc[id].sum / ADC_NB_SAMPLES) : RC_MAX_VALUE ;
	return (buf_adc[id].sum / ADC_NB_SAMPLES);
}

inline uint16_t get_rc_raw(int id)
{// return 0~RC_RANGE_VALUE
	float range;
	float tmp;
	uint16_t adc;
	if( id < NB_ADC ){
		range= get_adc(id)-adc_sensor_range[id][ADC_MIN_ID];
		if( range < 0 ) range = 0;
		range= RC_RANGE_VALUE*range;
		range = range/adc_sensor_range[id][ADC_RANGE_ID];
		if( range > RC_RANGE_VALUE ) range = RC_RANGE_VALUE;
		/*
		adc = get_adc(id);
		if( adc > adc_sensor_range[id][ADC_MAX_ID] ) adc = adc_sensor_range[id][ADC_MAX_ID];
		if( adc < adc_sensor_range[id][ADC_MIN_ID] ) adc = adc_sensor_range[id][ADC_MIN_ID];

		if( adc > adc_sensor_range[id][ADC_MIDDLE_ID] ){
			tmp = adc - adc_sensor_range[id][ADC_MIDDLE_ID];
			tmp = tmp * adc_sensor_again[id][1];
			tmp += 500;
		}else{
			tmp = adc;
			tmp = tmp * adc_sensor_again[id][0];
		}
		range = tmp;
		*/
	}else{
		range = 10;
	}
	return (uint16_t)range;
}

uint16_t get_rc(int id){ 
	if( id >=0 && id < RC_MAX_COUNT )
		return rc_value_list[id];
	return RC_MIN_VALUE;
}
inline uint16_t user_revert_rc_raw(int id, uint16_t rc)
{
	if( (rc_revert_mask & (1<<id)) != 0 ){
		return (RC_RANGE_VALUE - rc );	
	}
	return rc;
}
inline uint16_t user_curve_rc_raw(int id, uint16_t rc){
	short paramk,type;
	int tmp;
	paramk=rc_user_cuver[id][0];
	type=rc_user_cuver[id][1];

	if( paramk == 0 )
		return rc;
	
	if( type == CURVE_THR_TYPE){
		tmp = expo(rc,paramk);
	}else{
		tmp = (rc<<1) - RC_RANGE_VALUE;// 2*(rc-1022/2) 
		tmp = expo(tmp,paramk);

		tmp += RC_RANGE_VALUE; // tmp/2+1022/2
		tmp = tmp >> 1;
	}
	if( tmp > RC_RANGE_VALUE ) tmp = RC_RANGE_VALUE;
	if( tmp < 0) tmp = 0;
	return (uint16_t)tmp;

}
uint16_t user_zoom_rc(int id, uint16_t rc_raw){
	uint16_t min,max,rc_z;
	float rang;
	float tmp;
	min = rc_user_range[id][0];
	max = rc_user_range[id][1];
	if( min != 0 && max != 0 ){
		rang= max-min;
		tmp = rang/RC_RANGE_VALUE;
		tmp = tmp * rc_raw;
		rc_z = min+tmp;//rc_raw * ( rang/RC_RANGE_VALUE );
		//if( rc_z < RC_MIN_VALUE ) rc_z = RC_MIN_VALUE;
		if( rc_z > RC_MAX_VALUE ) rc_z = RC_MAX_VALUE;
		return rc_z;
	}else{
		return rc_raw+RC_MIN_VALUE;
	}
}

inline uint16_t cail_user_mix_rc(){
	int i,sid,mid;
	float again[RC_MAX_COUNT]={0};
	uint16_t middle_val=0;

	for( i=0 ; i < MIX_LIST_MAX_COUNT ; i++){
		if( mix_list[i].valiable == 1 ){
			sid = mix_list[i].slave_id;
			mid = mix_list[i].main_id;
			if( mix_list[i].start_position == 0){
				if( mix_list[i].add_persen != 0){
					again[sid] = mix_list[i].add_persen * (rc_value_list[mid]- get_rc_min_value(mid));
				}
			}else{
				middle_val = (get_rc_max_value(mid)-get_rc_min_value(mid))/2;
				middle_val += get_rc_min_value(mid);

				if( middle_val > get_rc(mid) ){
					if( mix_list[i].sub_persen != 0){
						again[sid] = -1*mix_list[i].sub_persen * (middle_val - get_rc(sid)); 
					}
				}else{
					if( mix_list[i].add_persen != 0){
						again[sid] = mix_list[i].add_persen * (get_rc(sid)-middle_val) ;
					}
				}
			}
		}
	}
	for( i=0 ; i < RC_MAX_COUNT ; i++){
		if( again[i] != 0 ){
			rc_value_list[i] += again[i]; 
		}
	}

}
inline uint16_t cail_user_trim_rc(){
	int i;
	for( i=0 ; i< RC_MAX_COUNT; i++){
		if( rc_user_trim[i] != 0 ){
			rc_value_list[i]+= rc_user_trim[i];
			if( rc_value_list[i] < RC_MIN_VALUE)
				rc_value_list[i] = RC_MIN_VALUE;
			if( rc_value_list[i] > RC_MAX_VALUE)
				rc_value_list[i] = RC_MAX_VALUE;
		}
	}
}

void update_rc()
{
	int i;
	uint16_t rc;

	for( i=0; i< RC_MAX_COUNT; i++){
		rc = get_rc_raw(i);	
		rc = user_revert_rc_raw(i,rc);
		rc = user_curve_rc_raw(i,rc);
		rc_value_list[i] = user_zoom_rc(i,rc);
	}
	//cail_user_mix_rc();
	//cail_user_trim_rc();
}


#define get_rc_for_mavlink(id) get_rc(id)
uint16_t get_rc_for_reciver(id)
{
	uint16_t rc = get_rc(id);
	
	if( rc < RC_RANGE_VALUE ){
		return 0;
	}else if( rc > RC_MAX_VALUE ){
		return RC_RANGE_VALUE;
	}else{
		return (rc - RC_RANGE_VALUE);
	}
}




#define ZFRAME_RCS_INT_COUNT 2 // RC_MAX_COUNT/3; as each int store 3 rc, (RC_MAX_COUNT/3)=2=int[2] ; (tag(16bit)+ 2*32bit+ crc(16bit)) = 10*8bit 
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
	uint32_t rcs[ZFRAME_RCS_INT_COUNT];//RC_MAX_COUNT/3 , rcs[0]=rc1-0-rc2-0-rc3 .... ; if reciver to sender  count =zframe_decode_false_count rc1= zframe_send_packet_count rc2 = zframe_recive_success_count
	uint16_t crc; //must put in the end
}zframe_packet_t;

#define ZFRAME_UART_BUFF_SIZE 1024
#define ZFRAME_SENDER
//#define ZFRAME_RECIVER
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
	/*
	if( zframe_get_count(packet) != RC_MAX_COUNT ){
		log("packet.count error =%d\n\r",zframe_get_count(packet));
		return -1;
	}
	*/
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
	for( i=0; i< RC_MAX_COUNT && rc_index <ZFRAME_RCS_INT_COUNT ; i++){
		rd[i]= (p32[rc_index]>>bit_cnt) & 0x3ff;	
		bit_cnt+=11;
		if( bit_cnt > 22 ){
			bit_cnt = 0;
			rc_index++;
		}
	}
	if( i < RC_MAX_COUNT ){
		log("decode_zframe rc count is not right\n\r");
		return -1;
	}
	return 0;
}
void encode_rcs(uint8_t *buff)
{
	int id,index,count;
	uint32_t *p32;
	uint16_t crc;

	//tag
	buff[0] = 0xff;buff[1] = 0x03;
	//count
	buff[2] = RC_MAX_COUNT;
	//rc
	p32 = (uint32_t *)&buff[3];
	memset(p32,0, sizeof(uint32_t)*ZFRAME_RCS_INT_COUNT);
	for( id=0,index=0,count=0; id < RC_MAX_COUNT; id++,count+=11){
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
void encode_status(uint8_t *buff,unsigned long recive_error_cnt, unsigned long recive_cnt, unsigned long send_cnt)
{
	int index;
	uint32_t *p32;
	uint16_t crc;

	//tag
	buff[0] = 0xff;buff[1] = 0x03;
	//count
	if( recive_error_cnt > 0xff ) 
		buff[2]=0xff;
	else
		buff[2] = recive_error_cnt ;
	//rc
	p32 = (uint32_t *)&buff[3];
	if( ZFRAME_RCS_INT_COUNT >= 2 ){
		p32[0] =send_cnt;
		p32[1]=recive_cnt;
	}
	//crc
	crc = crc_calculate( buff, ZFRAME_BUFF_SIZE-2);
	buff[ZFRAME_BUFF_SIZE-2]=crc&0xff;
	buff[ZFRAME_BUFF_SIZE-1]=(crc>>8)&0xff;
}
void collect_zframe(uint8_t *data, int len, void (*cb)(uint8_t * data))
{
	static uint8_t recive_buff[ZFRAME_BUFF_SIZE]={0};
	static int index0 = 0;

	int i;
	int packet_start = 0;

	//log("collect_zframe: start \n\r");
	for( i = 0; i< len; i++){
		if( index0 >= ZFRAME_BUFF_SIZE){
			cb(recive_buff);
			index0 = 0;
		}
		if( index0 == 1 ){
			if( is_zframe_tag(recive_buff[0],data[i]) ){
				recive_buff[index0++]=data[i];
			}else{
				index0 = 0;
			}
		}else{
			recive_buff[index0++]=data[i];
		}
	}
	//log("collect_zframe: end\n\r");
}
inline void send_data_by_uart(struct uart_periph *uartz, uint8_t*data, int len)
{
	int i;
	for( i = 0 ; i< len; i++)
		uart_put_byte(uartz, data[i]);
}

#ifdef ZFRAME_RECIVER

//################################################3 zframe reciver
#define zframeReciverUart &uart3
unsigned long zframe_recive_success_count=0;
//unsigned long zframe_recive_packet_count=0; //packget_count = success+false
unsigned long zframe_recive_decode_false_count=0;
unsigned long zframe_send_packet_count = 0;
int status_tid=0;

void handle_zframe_packet_from_sender(uint8_t * data)
{
	int error,i;
	uint16_t rc_data[RC_MAX_COUNT];


	error = decode_zframe(data,rc_data);

	log("get rc:\n\r");
	if( error == 0 ){
		zframe_recive_success_count ++;
		for(i=0 ; i< RC_MAX_COUNT; i++) 
			log("RC%d=%d,",i,rc_data[i]);

		//do_rc_to_pwm(rc_data,RC_MAX_COUNT);
	}else{
		zframe_recive_decode_false_count++;
	}
	log("...\n\r");
	//log("...\n\r,get packet=%d,false=%d",zframe_recive_success_count,zframe_decode_false_count);
}
void listen_sender_by_uart(struct uart_periph *uarts)
{
	int data_len,i;
	uint8_t buff[ZFRAME_UART_BUFF_SIZE];

	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++)
			buff[i] = uart_getch(uarts);	
		collect_zframe(buff,data_len,handle_zframe_packet_from_sender);
	}
}
void send_status_to_sender_by_uart()
{
	static uint8_t buff[ZFRAME_BUFF_SIZE];
	encode_status(buff, zframe_recive_decode_false_count , zframe_recive_success_count, zframe_send_packet_count);
	send_data_by_uart(zframeReciverUart,buff,ZFRAME_BUFF_SIZE);
	zframe_send_packet_count++;
}
void zframe_reciver_setup()
{
	ppz_pwm_setup();
	status_tid =sys_time_register_timer(1.0/4,NULL);		
}

void zframe_reciver_loop()
{
	listen_sender_by_uart(zframeReciverUart);

	if( sys_time_check_and_ack_timer(status_tid) )
		send_status_to_sender_by_uart();
log("zframe send %d packet;  recived ok %d, decode err %d\n\r",zframe_send_packet_count,zframe_recive_success_count,zframe_recive_decode_false_count );
delay_ms(10);
}
//################################################3 zframe reciver

#endif //ZFRAME_RECIVER




#ifdef ZFRAME_SENDER

//################################################ zframe sender
#define zframeSenderUart &uart2
#define configUart &uart3
unsigned long zframe_reciver_success_count=0; //the reciver's success count
unsigned long zframe_reciver_decode_false_count=0;//the reciver's false count
unsigned long zframe_reciver_send_count=0; // the reciver's send count

unsigned long zframe_send_packet_count = 0;//sender send count
unsigned long zframe_sender_recive_packet_success_count = 0;//sender reciver ok  count
unsigned long zframe_sender_recive_packet_false_count = 0;//sender reciver false count
int status_tid=0;
int sendrc_tid=0;
int view_rc_tid = 0;

void send_rc_to_reciver_by_uart()
{
	static uint8_t buff[ZFRAME_BUFF_SIZE]={0};

	encode_rcs(buff);
	send_data_by_uart(zframeSenderUart, buff, ZFRAME_BUFF_SIZE);
	zframe_send_packet_count++;
}
void display_status()
{
	log("zframe send %d packet;  recived ok %d, decode err %d\n\r",zframe_send_packet_count,zframe_reciver_success_count,zframe_reciver_decode_false_count);
	//log("zframe reciver send %d, recive ok %d, decode err %d, \n\r",zframe_reciver_send_count,zframe_sender_recive_packet_success_count,zframe_sender_recive_packet_false_count);
}

uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN];
void send_rc_to_user()
{
	int id=0;
	int len;
	mavlink_message_t msg;
	mavlink_rc_channels_override_t packet;

	packet.target_system=1;
	packet.target_component=1;
	packet.chan1_raw= get_rc_for_mavlink(id++);
	packet.chan2_raw= get_rc_for_mavlink(id++);
	packet.chan3_raw= get_rc_for_mavlink(id++);
	packet.chan4_raw= get_rc_for_mavlink(id++);
	packet.chan5_raw= get_rc_for_mavlink(id++);
	packet.chan6_raw= get_rc_for_mavlink(id++);
	packet.chan7_raw= get_rc_for_mavlink(id++);
	packet.chan8_raw= get_rc_for_mavlink(id++);

	mavlink_msg_rc_channels_override_encode(255, 109,  &msg, &packet);
	len = mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
	send_data_by_uart(configUart,mavlink_buffer,len);
}
void handle_zframe_packet_frome_reciver(uint8_t *data)
{
	int error,i;
	uint16_t rc_data[RC_MAX_COUNT];
	uint32_t *p;

	error = check_zframe_packet(data);
	if( error == 0 ){
		zframe_sender_recive_packet_success_count++;

		p = zframe_get_rcs_point(data);
		zframe_reciver_success_count = p[1];  
		zframe_reciver_send_count = p[0];  
		zframe_reciver_decode_false_count = zframe_get_count(data); 

	}else{
		zframe_sender_recive_packet_false_count++;
	}
}
void listen_reciver_by_uart(struct uart_periph *uarts)
{
	int data_len,i;
	uint8_t buff[ZFRAME_UART_BUFF_SIZE];

	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++)
			buff[i] = uart_getch(uarts);	
		collect_zframe(buff,data_len,handle_zframe_packet_frome_reciver);
	}
}

void jostick_led_init()
{
        gpio_set_mode(LED_STATUS_GPIO_BANK, GPIO_MODE_OUTPUT_2_MHZ,
                      GPIO_CNF_OUTPUT_PUSHPULL,  LED_STATUS_GPIO);
        gpio_clear(LED_STATUS_GPIO_BANK, LED_STATUS_GPIO);
}
inline void jostick_led_on(){
        gpio_set(LED_STATUS_GPIO_BANK, LED_STATUS_GPIO);
}
inline void jostick_led_off(){
        gpio_clear(LED_STATUS_GPIO_BANK, LED_STATUS_GPIO);
}
inline void jostick_led_toggle(){
        gpio_toggle(LED_STATUS_GPIO_BANK, LED_STATUS_GPIO);
}



#if USE_HANDSET

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
volatile static int handset_irq_counter = 0;
volatile static uint16_t pin_status = 0; 
uint16_t irq_error_count1 = 0; 
uint16_t irq_error_count2 = 0; 
uint16_t irq_error_count3 = 0; 
uint16_t irq_error_count4 = 0; 

void reset_handset_count()
{
	pin_status = gpio_port_read(GPIOB)& (GPIO4|GPIO5);
	handset_irq_counter = 0;
}
void update_pin_status_AB_step_mode() // AB: 00->10->11->01 -> 00 -> 10 ....  
{
	uint16_t new_pins = gpio_port_read(GPIOB)& (GPIO4|GPIO5);
	
	uint16_t new_A = new_pins & GPIO4;
	uint16_t new_B = new_pins & GPIO5;

	uint16_t old_A = pin_status & GPIO4;
	uint16_t old_B = pin_status & GPIO5;

	if( old_A == new_A ){
		if( new_A == 0 ){
			if( new_B < old_B ){
				handset_irq_counter++;
			}else if( new_B > old_B ){
				handset_irq_counter--;
			}else{
				irq_error_count1++;		
			}
		}else{
			if( new_B > old_B ){
				handset_irq_counter++;
			}else if( new_B < old_B ){
				handset_irq_counter--;
			}else{
				irq_error_count2++;		
			}
		}

	}else{
		if( old_B != new_B ) {
			irq_error_count3++;
		}else{
			if( new_A < old_A ){
				if( new_B > 0 )
					handset_irq_counter++;
				else
					handset_irq_counter--;
			}else if( new_A > old_A){
				if( new_B > 0 )
					handset_irq_counter--;
				else
					handset_irq_counter++;
			}else{
				irq_error_count4++;
			}
		}

	}
	pin_status = new_pins;
	
}

void update_pin_status()
{
	uint16_t new_pins = gpio_port_read(GPIOB)& (GPIO4|GPIO5);
	
	uint16_t new_A = new_pins & GPIO4;
	uint16_t new_B = new_pins & GPIO5;

	uint16_t old_A = pin_status & GPIO4;
	uint16_t old_B = pin_status & GPIO5;

	if( new_pins == pin_status ) return ;
	
	if( new_pins == 0 ){
		if( (pin_status&GPIO4) != 0 ){//gpio4 is fall , the direct is left/right
			handset_irq_counter++;
		}else if( (pin_status&GPIO5) != 0){
			handset_irq_counter--;
		}
	}
	pin_status = new_pins;
}

void exti4_isr(void)
{
	update_pin_status();
	//log ("4= %x \r\n",gpio_port_read(GPIOB)& (GPIO4|GPIO5));
	exti_reset_request(EXTI4);
}
void exti9_5_isr(void)
{
	update_pin_status();
	//log ("5= %x \r\n",gpio_port_read(GPIOB)& (GPIO4|GPIO5));
	exti_reset_request(EXTI5);

}
static void handset_exti_setup(void)
{
	/* Enable GPIOA clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable AFIO clock. */
	rcc_periph_clock_enable(RCC_AFIO);

	/* Enable EXTI0 interrupt. */
	nvic_enable_irq(NVIC_EXTI4_IRQ);
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);

	/* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
	gpio_clear(GPIOB,GPIO4);
	gpio_clear(GPIOB,GPIO5);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4);//GPIO_CNF_INPUT_FLOAT //GPIO_CNF_INPUT_PULL_UPDOWN
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5);//GPIO_CNF_INPUT_FLOAT //GPIO_CNF_INPUT_PULL_UPDOWN

	/* Configure the EXTI subsystem. */
	exti_select_source(EXTI4, GPIOB);
	exti_select_source(EXTI5, GPIOB);
	//exti_direction = FALLING;
	exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING);//EXTI_TRIGGER_BOTH);
	exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
	//exti_reset_request(EXTI5);
	reset_handset_count();
	exti_enable_request(EXTI4);
	exti_enable_request(EXTI5);
}

#endif // USE_HANDSET



void zframe_sender_setup()
{
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

	//57600: 1/150  115200: 1/500 2250000: 1000
	sendrc_tid = sys_time_register_timer(1.0/100,NULL);	
	//sendrc_tid = sys_time_register_timer(1.0/1000,NULL);	
	status_tid =sys_time_register_timer(1.0,NULL);		
	view_rc_tid =sys_time_register_timer(1.0/50,NULL);		

	mcu_int_enable();

	adc_sensor_cali_init();

	init_rc_mix_list();

	jostick_led_init();

#if USE_HANDSET
	handset_exti_setup();
#endif
}

void zframe_sender_loop()
{
	static float t1,t2;
	//if(uart_check_free_space(zframeSenderUart,ZFRAME_BUFF_SIZE)) 
		//send_rc_to_reciver_by_uart();

	//listen_reciver_by_uart(zframeSenderUart);
	do_copy_uart_and_handle_mavlink_msg(configUart,NULL);

	if( sys_time_check_and_ack_timer(status_tid) ){
		display_status();
	}
	
	if( sys_time_check_and_ack_timer(sendrc_tid) ){
		//t1 = get_sys_time_float();
		update_rc();
		#if USE_HANDSET
		rc_value_list[RC_THR_ID] = RC_MIN_VALUE + handset_irq_counter*10;
		if( rc_value_list[RC_THR_ID] < RC_MIN_VALUE ) rc_value_list[RC_THR_ID]=RC_MIN_VALUE;
		if( rc_value_list[RC_THR_ID] > RC_MAX_VALUE ) rc_value_list[RC_THR_ID]=RC_MAX_VALUE;
		#endif
		send_rc_to_reciver_by_uart();
		//t2 = get_sys_time_float();
		//log("send time : %f\r\n",t1-t2);
		
		/*
		for( int i=0; i< RC_MAX_COUNT; i++)
			//log("%d,",get_adc(i));
			log("%d,",get_rc(i));
		log("\r\n");
		*/
	
		//log("handset_irq_counter=%d,pin=%x, %d,%d,%d,%d,\r\n",handset_irq_counter,pin_status,irq_error_count1,irq_error_count2,irq_error_count3,irq_error_count4);
	}

	if( sys_time_check_and_ack_timer(view_rc_tid) ){
		send_rc_to_user();
		jostick_led_toggle();
	}
/*
	do_copy_uart_data_to_other_uart(zframeSenderUart,userUart);
	do_copy_uart_data_to_other_uart(userUart, zframeSenderUart);
*/	
}
//################################################ zframe sender

#endif// ZFRAME_SENDER





//#################################################### main 

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

#ifdef ZFRAME_SENDER
	zframe_sender_setup();
#endif
#ifdef ZFRAME_RECIVER
	zframe_reciver_setup();
#endif
}

inline void loop()
{
#ifdef ZFRAME_SENDER
	zframe_sender_loop();
#endif
#ifdef ZFRAME_RECIVER
	zframe_reciver_loop();
#endif

#if 0
	log("get rc:");
	for( int i=0 ; i< 6; i++){
	//	log("rc%d=%d,",i,get_rc_for_mavlink(i));
		log("rc%d=%d,",i,get_rc_for_reciver(i));
		log("%d,,",get_adc(i));
	}
	log("\n\r");
	delay_ms(100);

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

