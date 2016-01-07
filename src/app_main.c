#include <mcu_periph/uart.h>
#include <mcu.h>
#include "mcu_periph/sys_time.h"
#include <mavlink.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <subsystems/actuators/actuators_pwm_arch.h>

#define CONSOLE_UART &uart1
//using now
#define COPTER_UART &uart2
#define REMOTE_4G_UART &uart1
#define REMOTE_2G_UART &uart3
//no use
#define TELEM_UART &uart3
#define SBUS_PPM_UART &uart2

#define COPTER_COM_ID MAVLINK_COMM_0
#define REMOTE_4G_COM_ID MAVLINK_COMM_1
#define TELEM_COM_ID MAVLINK_COMM_2
//#define SBUS_PPM_COM_ID MAVLINK_COMM_1

#define DEBUG_APP 0
#define log(format, ...) if( DEBUG_APP ) printf(format, ## __VA_ARGS__)

#define CHANNELS_MAX_COUNT ACTUATORS_PWM_NB





enum rc_autopilot_modes {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    OF_LOITER =    10,  // deprecated
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17   // full-brake using inertial/GPS system, no pilot input
};
#define   ERROR_MODE  -1

#define CHAN_COUNT 8
#define ROLL_ID 0
#define PITCH_ID 1
#define THR_ID 2
#define YAW_ID 3
#define CHAN_5_ID 4
#define CHAN_6_ID 5
#define CHAN_7_ID 6
#define CHAN_8_ID 7
#define MAVLINK_SYSID 255
#define MAVLINK_COMPID 190

uint16_t copter_speed = 200;
#define ROLL_LEFT_DTMF '0'
#define ROLL_RIGHT_DTMF  '1'
#define PITCH_UP_DTMF  '2'
#define PITCH_DOWN_DTMF  '3'
#define THR_UP_DTMF  '4'
#define THR_DOWN_DTMF  '5'
#define YAW_LEFT_DTMF '6'
#define YAW_RIGHT_DTMF  '7'
#define STOP_DTMF  '8'
#define ARM_DTMF  '9'
#define LAN_DTMF  '9'
#define TAKEOFF_DTMF  '*'
#define SPEED_ADD_DTMF  'A'
#define SPEED_SUB_DTMF  'B'

uint16_t mavlink_rcs[8]={1000};
uint8_t is_armed = 0;
uint8_t is_takeoff = 0;
uint8_t is_holdon = 0;
int copter_sysid = 0;
int copter_compid = 0;
mavlink_heartbeat_t g_current_heartbeat;

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

inline uint8_t get_uart_mavlink_com(struct uart_periph *uarts)
{
	uint8_t com = MAVLINK_COMM_0;
	if( uarts == &uart1 ) com = MAVLINK_COMM_0; 
	else if( uarts == &uart2 ) com = MAVLINK_COMM_1; 
	else if( uarts == &uart3 ) com = MAVLINK_COMM_2; 
	return com;
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
void do_mavlink_rc_override_to_pwm(mavlink_message_t *msg)
{
	uint16_t rcs[8];
	if(msg->msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
		rcs[0] = mavlink_msg_rc_channels_override_get_chan1_raw(msg);
		rcs[1] = mavlink_msg_rc_channels_override_get_chan2_raw(msg);
		rcs[2] = mavlink_msg_rc_channels_override_get_chan3_raw(msg);
		rcs[3] = mavlink_msg_rc_channels_override_get_chan4_raw(msg);
		rcs[4] = mavlink_msg_rc_channels_override_get_chan5_raw(msg);
		rcs[5] = mavlink_msg_rc_channels_override_get_chan6_raw(msg);
		rcs[6] = mavlink_msg_rc_channels_override_get_chan7_raw(msg);
		rcs[7] = mavlink_msg_rc_channels_override_get_chan8_raw(msg);
		do_rc_to_pwm(rcs,CHANNELS_MAX_COUNT);
	}
}
void do_copy_uart_and_handle_mavlink_msg(struct uart_periph *uarts ,struct uart_periph *uartd, void (*handler)(mavlink_message_t *msg) )
{
	mavlink_message_t msg; 
	mavlink_status_t status;
	int data_len,i;
	uint8_t buff[UART_RX_BUFFER_SIZE];
	uint8_t com = get_uart_mavlink_com(uarts);

	data_len = uart_char_available(uarts);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++)
			buff[i] = uart_getch(uarts);
	
		if( uartd != NULL ){
			for( i = 0 ; i< data_len; i++)
				uart_put_byte(uartd, buff[i] );
		}

		for( i = 0 ; i< data_len; i++){
			if( mavlink_parse_char(com, buff[i], &msg, &status) ) { 
      				handler(&msg);
    			}
		}
  	}
}
void do_sbus_send_pwm(int32_t * rcs, int count)
{
	px4_send_sbus_data(rcs,count);
}


//###########################################################################################################
#define REMOTE_2G_BUFF_SIZE 128
uint8_t remote_2g_recived_buffer[REMOTE_2G_BUFF_SIZE]={0};
int remote_2g_buff_index = 0;
uint8_t current_2g_cmd[REMOTE_2G_BUFF_SIZE];
uint8_t is_2g_cmd_finished = 1;
uint8_t is_2g_cmd_ok = 0;

uint8_t is_2g_inited_ok = 0;
uint8_t is_2g_connected = 0;
uint8_t is_2g_at_test_ok = 0;
float last_heartbeat_time = 0;
//cmd
#define ACCEPT_CALL_STRING "ATA\r\n"  //if ok start send rc
#define SET_DTMF_ECHO_STRING "AT+DDET=1\r\n" //if ok  ,start listen call
#define TEST_AT_OK "AT\r\n"
//result
#define OK_STRING "OK\r\n"
#define ERROR_STRING "ERROR\r\n"
//message
#define CALLING_STRING "RING\r\n" // if start listen call , accept it
#define BREAK_CALL_STRING "NO CARRIER\r\n" // set mode guied and stop send rc
#define DTMF_HEAD_STRING "+DTMF:" //+DTMF:3


void clear_mavlink_rc_value()
{
	mavlink_rcs[ROLL_ID] = 1500;
	mavlink_rcs[PITCH_ID] = 1500;
	mavlink_rcs[YAW_ID] = 1500;
	mavlink_rcs[THR_ID] = 1000;
	mavlink_rcs[CHAN_5_ID]=1000;
	mavlink_rcs[CHAN_6_ID]=1000;
	mavlink_rcs[CHAN_7_ID]=1000;
	mavlink_rcs[CHAN_8_ID]=1000;
}

int is_copter_connected()
{
	float now_time = get_sys_time_float();
	
	if( now_time - last_heartbeat_time > 5 )
		return 0;
	else 
		return 1;
}

int is_copter_armed()
{
	if( is_copter_connected() == 0 )
		return 0;
  if( (g_current_heartbeat.base_mode & (uint8_t)MAV_MODE_FLAG_SAFETY_ARMED) == 0 )
    return 0;
  else
    return 1;
}
int get_copter_mode()
{
  if( is_copter_connected() )
  {
    return g_current_heartbeat.custom_mode;
  }
  return ERROR_MODE;
}
inline void send_mavlink_message(struct uart_periph *uarts, mavlink_message_t *message)
{
	int ret,i;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, message);

	for( i = 0 ; i< len; i++)
		uart_put_byte(uarts, buf[i] );
	return ;
}
void send_rc_override_messages(struct uart_periph *uarts, uint16_t *rcs)
{
  int ret = 0;
  mavlink_rc_channels_override_t sp;
  mavlink_message_t message;
  
  
// fill with the sp 
  sp.chan1_raw = rcs[ROLL_ID];
  sp.chan2_raw = rcs[PITCH_ID];
  sp.chan3_raw = rcs[THR_ID];
  sp.chan4_raw = rcs[YAW_ID];
  sp.chan5_raw = rcs[CHAN_5_ID];
  sp.chan6_raw = rcs[CHAN_6_ID];
  sp.chan7_raw = rcs[CHAN_7_ID];
  sp.chan8_raw = rcs[CHAN_8_ID];
  sp.target_component = 1;
  sp.target_system = 1;
  

  mavlink_msg_rc_channels_override_encode(MAVLINK_SYSID, MAVLINK_COMPID ,&message, &sp);
  send_mavlink_message(uarts,&message);
}

void send_setmode_message(int mode)
{
  mavlink_set_mode_t mode_sp;
  mavlink_message_t message;
  int ret;
  
	mode_sp.base_mode = (uint8_t)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;// g_current_heartbeat.base_mode;// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  ;// MAV_MODE_FLAG_DECODE_POSITION_SAFETY ;
  mode_sp.custom_mode = (uint32_t)mode;
  mavlink_msg_set_mode_encode(copter_sysid, copter_compid, &message , &mode_sp);
  //log("set mode sysid=%d,compid=%d\r\n",copter_sysid,copter_compid);
  send_mavlink_message(COPTER_UART,&message);
  //mavlink_msg_set_mode_encode(250, 190, &message , &mode_sp);
  //send_mavlink_message(COPTER_UART,&message);
  
}

void send_arm_disarm_message(int arm)
{
  mavlink_command_long_t sp;
  mavlink_message_t message;
  int ret;
    
	sp.command = (uint16_t)MAV_CMD_COMPONENT_ARM_DISARM;
	sp.target_system = (uint8_t)MAVLINK_SYSID;//control_data.system_id;
	sp.target_component = (uint8_t)MAVLINK_COMPID; 
  sp.param1= (float)arm;

  mavlink_msg_command_long_encode(copter_sysid, copter_compid  ,&message,&sp);
	send_mavlink_message(COPTER_UART,&message);
}

void send_takeoff_message(int m)
{
  mavlink_command_long_t sp;
  mavlink_message_t message;
  int ret;
    
	sp.command = (uint16_t)MAV_CMD_NAV_TAKEOFF;
	sp.target_system = (uint8_t)MAVLINK_SYSID;//control_data.system_id;
	sp.target_component = (uint8_t)MAVLINK_COMPID; 
  sp.param3= 0;//(float)arm;
  sp.param7= m;//(float)arm;//10 m

  mavlink_msg_command_long_encode(copter_sysid, copter_compid  ,&message,&sp);
 send_mavlink_message(COPTER_UART,&message);
}

void send_land_message()
{

  mavlink_command_long_t sp;
  mavlink_message_t message;
  int ret;
    
	sp.command = (uint16_t)MAV_CMD_NAV_LAND;
	sp.target_system = (uint8_t)MAVLINK_SYSID;//control_data.system_id;
	sp.target_component = (uint8_t)MAVLINK_COMPID; 

  mavlink_msg_command_long_encode(copter_sysid, copter_compid  ,&message,&sp);
send_mavlink_message(COPTER_UART,&message);
}

void sync_g_heartbeat_message(mavlink_message_t *msg)
{
	if( msg->msgid != MAVLINK_MSG_ID_HEARTBEAT )
		return;
	g_current_heartbeat.custom_mode = mavlink_msg_heartbeat_get_custom_mode(msg);
	g_current_heartbeat.type = mavlink_msg_heartbeat_get_type(msg);
	g_current_heartbeat.autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
	g_current_heartbeat.base_mode = mavlink_msg_heartbeat_get_base_mode(msg);
	g_current_heartbeat.system_status = mavlink_msg_heartbeat_get_system_status(msg);
	g_current_heartbeat.mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
	copter_sysid = msg->sysid;
	copter_compid = msg->compid;

	last_heartbeat_time = get_sys_time_float();
#if DEBUG_APP
	log("get a heart mavlink msg\r\n");
	if( is_2g_at_test_ok != 1 )send_2g_cmd_string(TEST_AT_OK);
#endif
}


inline void do_send_rc_override_to_copter()
{
	if( is_2g_connected )
		send_rc_override_messages(COPTER_UART,mavlink_rcs);
}
void do_arm_key_event()
{
	log("do_arm_key_event\r\n");

	clear_mavlink_rc_value();
	do_send_rc_override_to_copter();
	delay_ms(20);
	do_send_rc_override_to_copter();
	delay_ms(20);
	send_setmode_message(GUIDED);
	delay_ms(50);
	if( is_copter_armed() == 1 ){
		send_arm_disarm_message(0);
	}else{
		//delay_ms(20);
		send_arm_disarm_message(1);
	}
}
void do_takeoff_key_event()
{
	int count = 0;

	log("do_takeoff_key_event..");
	if( is_copter_connected()==0 )
		return;
	log("setmode...");
	while( count++ < 30 && GUIDED != get_copter_mode() ){
		delay_ms(100);
		send_setmode_message(GUIDED);
	}
	if( GUIDED != get_copter_mode() ){
		log("false\r\n");
		return ;
	}

	clear_mavlink_rc_value();
	do_send_rc_override_to_copter();
	//delay_ms(50);
	log("arm...");
	count = 0;
	while( count++ < 10 && is_copter_armed() == 0 ){
		send_arm_disarm_message(1);
		delay_ms(300);
	}
	if( is_copter_armed() == 0 ){
		send_arm_disarm_message(0);
		log("false\r\n");
		return ;
	}
	send_takeoff_message(10);
	log("ok\r\n");
}
void do_holdon_key_event()
{
	int count=0;
	//delay_ms(50);
	//send_setmode_message(LOITER);
	log("do_holdon_key_event...");
	while( count++ < 20 && GUIDED != get_copter_mode() ){
		delay_ms(100);
		send_setmode_message(GUIDED);
	}
	if( GUIDED != get_copter_mode() )
		return;
	clear_mavlink_rc_value();
	mavlink_rcs[THR_ID]=1500;
	do_send_rc_override_to_copter();

	log("OK \r\n");
}
void do_return_key_event()
{
	//clear_mavlink_rc_value();
	//do_send_rc_override_to_copter();
	//delay_ms(50);
	log("send rtl mode\r\n");
	send_setmode_message(RTL);
}
int check_loiter_mode()
{
	int count=0;
	if( mavlink_rcs[THR_ID] == 1000 )
		mavlink_rcs[THR_ID] == 1500;
	while( count++ < 20 && LOITER != get_copter_mode() ){
		delay_ms(100);
		send_setmode_message(LOITER);
	}
	if( LOITER != get_copter_mode() ){
		mavlink_rcs[THR_ID] == 1000 ;
		log("check_loiter_mode false \r\n");
		return 0;
	}
	log("check_loiter_mode ok \r\n");
	return 1;
}
void handle_2g_key_event(char key_id)
{
	int need_check_mode = 0;
	log("get 2g key=%c\r\n",key_id);
	switch( key_id ){
		case LAN_DTMF:
			send_land_message();
			break;
		case TAKEOFF_DTMF:
			do_takeoff_key_event();
			break;
		case STOP_DTMF:
			do_holdon_key_event();
			break;
		case ROLL_LEFT_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[ROLL_ID]=1500-copter_speed;
			break;
		case ROLL_RIGHT_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[ROLL_ID]=1500+copter_speed;
			break;
		case YAW_LEFT_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[YAW_ID]=1500-copter_speed;
			break;
		case YAW_RIGHT_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[YAW_ID]=1500+copter_speed;
			break;
		case PITCH_UP_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[PITCH_ID]=1500+copter_speed;
			break;
		case PITCH_DOWN_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[PITCH_ID]=1500-copter_speed;
			break;
		case THR_UP_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[THR_ID]=1500+copter_speed;
			break;
		case THR_DOWN_DTMF:
			if( check_loiter_mode() )
				mavlink_rcs[THR_ID]=1500-copter_speed;
			break;

		case SPEED_ADD_DTMF:
			copter_speed+=50;
			if( copter_speed >= 500 ) copter_speed = 500;
			log("copter_speed=%d\r\n",copter_speed);
			break;
		case SPEED_SUB_DTMF:
			copter_speed-=50;
			if( copter_speed <= 10 ) copter_speed = 0;
			log("copter_speed=%d\r\n",copter_speed);
			break;
	}
}



inline int is_string(char *s, char *d)
{
	if( 0 == strcmp(s,d) ) {
		return 1;
	}else{
		return 0;
	}

}
void send_2g_cmd_string(char* cmd)
{
	int i;
	int len = strlen(cmd);

	log("send cmd: %s \r\n",cmd);
	//memcpy(current_2g_cmd,cmd,len);
	for( i=0; i< len; i++) 
		uart_put_byte(REMOTE_2G_UART,cmd[i]);
		//uart_put_byte(REMOTE_2G_UART,current_2g_cmd[i]);
	//is_2g_cmd_finished = 0;
}
void handle_2g_cmd_result()
{
	if( 1 == is_string(current_2g_cmd, SET_DTMF_ECHO_STRING) ) {
		log("SET_DTMF_ECHO_STRING cmd res=%d\r\n",is_2g_cmd_ok);
		is_2g_inited_ok = 0;
		if( is_2g_cmd_ok ) is_2g_inited_ok = 1;
	}else if( 1 == is_string(current_2g_cmd, ACCEPT_CALL_STRING) ) {
		log("ACCEPT_CALL_STRING cmd res=%d\r\n",is_2g_cmd_ok);
		is_2g_connected  = 0;
		if( is_2g_cmd_ok ) is_2g_connected = 1;
	}else if( 1 == is_string(current_2g_cmd,TEST_AT_OK) ){
		log("AT cmd res=%d\r\n",is_2g_cmd_ok);
		if( is_2g_cmd_ok ) is_2g_at_test_ok = 1;
	}
}
void handle_2g_one_line(uint8_t *buff, int len)
{
	int i;
	log("%s",buff);
	if( is_string(buff, OK_STRING) ) {
		//is_2g_cmd_finished = 0;
		is_2g_cmd_ok = 1;
		handle_2g_cmd_result();
	}else if( is_string(buff,ERROR_STRING) ) {
		//is_2g_cmd_finished = 0;
		is_2g_cmd_ok = 0;
		handle_2g_cmd_result();
	}else if( is_string(buff, ACCEPT_CALL_STRING) || is_string(buff,SET_DTMF_ECHO_STRING) || is_string(buff,TEST_AT_OK) ) {
		memcpy(current_2g_cmd,buff,len);
		current_2g_cmd[len]=0;
		//log("current cmd=%s,len=%d\r\n",current_2g_cmd,len);
	}else if( is_string(buff, CALLING_STRING) ){
		if( is_2g_inited_ok == 1 ){ 
			send_2g_cmd_string(ACCEPT_CALL_STRING);
		}else{
			send_2g_cmd_string(SET_DTMF_ECHO_STRING);
		}
	}else if( is_string( buff , BREAK_CALL_STRING) ){
		log("call break\r\n");
		is_2g_connected = 0;
		for( i=0; i< 10; i++){
			if( GUIDED == get_copter_mode(0) )
				break;
			send_setmode_message(GUIDED);
			delay_ms(200);
		}
	}else if( len > 8 &&  (0 == strncmp(buff, DTMF_HEAD_STRING, 6)) ) {
		handle_2g_key_event(buff[6]);
	}
}
void do_listen_2G()
{
	int data_len,i;
	uint8_t buff[UART_RX_BUFFER_SIZE];

	data_len = uart_char_available(REMOTE_2G_UART);
	if( data_len > 0 ){
		for( i = 0 ; i< data_len; i++)
			buff[i] = uart_getch(REMOTE_2G_UART);

#if DEBUG_APP
		for( i = 0 ; i< data_len; i++)
			uart_put_byte(CONSOLE_UART,buff[i]);
#endif
	
		for( i = 0; i< data_len; i++){
			if( buff[i] == '\n' ){
				remote_2g_recived_buffer[remote_2g_buff_index++]=buff[i];
				remote_2g_recived_buffer[remote_2g_buff_index]=0;
				handle_2g_one_line(remote_2g_recived_buffer,remote_2g_buff_index);
				remote_2g_buff_index = 0;
			}else{
				remote_2g_recived_buffer[remote_2g_buff_index++]=buff[i];
			}
			//clear such long line , it may be error
			if( remote_2g_buff_index >= 128 )  
				remote_2g_buff_index = 0;
		}
	}		
}
void do_listen_copter_for_2g()
{
	//do_copy_uart_and_handle_mavlink_msg(COPTER_UART,NULL,sync_g_heartbeat_message);

	do_copy_uart_and_handle_mavlink_msg(COPTER_UART,REMOTE_4G_UART,sync_g_heartbeat_message);
	//do_copy_uart_data_to_other_uart(COPTER_UART,REMOTE_4G_UART);
	do_copy_uart_data_to_other_uart(REMOTE_4G_UART,COPTER_UART);

}



void loop_4g_and_telem_to_copter()
{
	do_copy_uart_data_to_two_uart(COPTER_UART,REMOTE_4G_UART,TELEM_UART);
	do_copy_uart_data_to_other_uart(REMOTE_4G_UART,COPTER_UART);
	do_copy_uart_data_to_other_uart(TELEM_UART,COPTER_UART);
	//do_copy_uart_and_handle_mavlink_msg(TELEM_UART,COPTER_UART,do_mavlink_rc_override_to_pwm);
}
void loop_telem_and_sbusppm_to_copter()
{
	do_copy_uart_data_to_other_uart(COPTER_UART,TELEM_UART);
	do_copy_uart_and_handle_mavlink_msg(TELEM_UART,COPTER_UART,do_mavlink_rc_override_to_pwm);
	do_sbus_send_pwm(actuators_pwm_values,CHANNELS_MAX_COUNT);
	;//mavlink_parse_char
}
inline void loop_4g_and_2g_to_copter()
{
	do_copy_uart_data_to_other_uart(REMOTE_4G_UART,COPTER_UART);
	do_copy_uart_data_to_other_uart(COPTER_UART,REMOTE_4G_UART);
	do_listen_2G();
}

void setup_2G_data_init()
{
	is_armed = 0;
	is_takeoff = 0;
	is_holdon = 0;
	copter_sysid = 0;
	copter_compid = 0;
	//memset(g_current_heartbeat,0,sizeof(mavlink_heartbeat_t));	
	clear_mavlink_rc_value();
}



int sendrc_tid = 0;
int copter_tid = 0;

void setup()
{
	//mcu_init(); //define PERIPHERALS_AUTO_INIT in makefile , and will init preiph auto in mcu_init(), like uartx_init()
  mcu_arch_init();
  board_init();
  sys_time_init();
#if USE_UART1
  uart1_init();
#endif
#if USE_UART2
  uart2_init();
#endif
#if USE_UART3
  uart3_init();
#endif

	//pwm_timer_setup();
	ppz_pwm_setup();
	//uart define config in makefile and uart_arch.c 
  	//uart_periph_set_mode(&uart1, USE_UART1_TX, USE_UART1_RX, UART1_HW_FLOW_CONTROL);
  	//uart_periph_set_bits_stop_parity(&uart1, UART1_BITS, UART1_STOP, UART1_PARITY);
  	//uart_periph_set_baudrate(&uart1, UART1_BAUD);


	setup_2G_data_init();
	sendrc_tid = sys_time_register_timer(1.0/50,NULL);
	copter_tid = sys_time_register_timer(1.0/200,do_listen_copter_for_2g);
	log("start app\r\n");
}
inline void loop()
{
	//loop_4g_and_telem_to_copter();
	//loop_telem_and_sbusppm_to_copter();


	do_listen_2G();
	if( sys_time_check_and_ack_timer(sendrc_tid) )
		do_send_rc_override_to_copter();

	//do_copy_uart_data_to_other_uart(COPTER_UART, REMOTE_4G_UART);
	//do_copy_uart_data_to_other_uart(REMOTE_4G_UART, COPTER_UART);

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

