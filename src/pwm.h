#ifndef _PWM_H
#define _PWM_H

#include <mcu.h>
#include "mcu_periph/sys_time.h"
#include <stdio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <subsystems/actuators/actuators_pwm_arch.h>


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



#endif
