#ifndef _PWM_H
#define _PWM_H

#include <mcu.h>
#include "mcu_periph/sys_time.h"
#include <stdio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <subsystems/actuators/actuators_pwm_arch.h>


//#define log(format, ...) if( 1 ) printf(format, ## __VA_ARGS__)
void pwm_setup();
void pwm_stop();
void pwm_start();
inline void pwm_recovery_rcs(uint16_t *rcs, int count)
{
	int id,cnt;
	cnt = count > ACTUATORS_PWM_NB ? ACTUATORS_PWM_NB:count;
	for( id = 0; id<cnt; id ++){
		ActuatorPwmSet(id,rcs[id]);
	}
	//actuators_pwm_commit();
}
inline void pwm_recovery_part_rcs(uint16_t *rcs, int count, int start_idx)
{
	int id,i;

#if 0
	log("start=%d\r\n",start_idx);
#endif
	for( i=0,id=start_idx; id<ACTUATORS_PWM_NB && i<count; id++,i++){
		ActuatorPwmSet(id,rcs[i]);
#if 0
		log("%d,",rcs[i]);
#endif
	}
#if 0
	log("\r\n");
#endif
	//actuators_pwm_commit();
}


#endif
