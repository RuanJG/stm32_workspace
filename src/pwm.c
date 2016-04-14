#include "pwm.h"

/*
 *
 * *********************************** PWM function 
 *
 */
uint8_t pwm_enable =0;
uint16_t pwm_period_us = 20000;
uint16_t pwm_period_update_us = 17000;
int pwm_t=0;

void pwm_start()
{
	pwm_enable = 1;
	actuators_pwm_start();
#if USE_PWM_LED
	PWM_LED_ON();
#endif
}
void pwm_stop()
{
	//TODO need to recovery rc first ?
	actuators_pwm_stop();
	pwm_enable = 0;
#if USE_PWM_LED
	PWM_LED_OFF();
#endif
}


void pwm_task()
{
	if( pwm_enable && actuators_pwm_get_max_counter() >= pwm_period_update_us ){
		actuators_pwm_commit();
		#if USE_PWM_LED
		PWM_LED_TOGGLE();
		#endif
	}
}


#define PWM_LOOP_USE_SYSTICK_INT 1
void pwm_setup()
{
	int id;
	for( id =0 ; id < ACTUATORS_PWM_NB ; id++ )
		ActuatorPwmSet(id,1001);

	
	#if USE_PWM_LED
	gpio_setup_output(PWM_LED_GPIO,PWM_LED_PIN);
	PWM_LED_OFF();
	#endif

	actuators_pwm_arch_init();
	actuators_pwm_commit();

	pwm_stop();
	pwm_period_us = 1000000/SERVO_HZ; // us for one pwm frame; 
	pwm_period_update_us = pwm_period_us - PWM_UPDATE_OC_VALUE_DIFF_US; 

#if PWM_LOOP_USE_SYSTICK_INT
	pwm_t = sys_time_register_timer(0.001, pwm_task);
#else
	pwm_t = sys_time_register_timer(0.001, NULL);// pwm_task);
#endif
}

#if 0
void pwm_loop()
{
	if( sys_time_check_and_ack_timer(pwm_t) ){
		pwm_task();
	}
}
#endif


