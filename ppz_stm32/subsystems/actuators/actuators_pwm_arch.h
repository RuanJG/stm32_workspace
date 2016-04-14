/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/stm32/subsystems/actuators/actuators_pwm_arch.h
 *  STM32 PWM servos handling.
 */

#ifndef ACTUATORS_PWM_ARCH_H
#define ACTUATORS_PWM_ARCH_H

#include "mcu_periph/std.h"

#include BOARD_CONFIG

#ifndef ACTUATORS_PWM_NB
#define ACTUATORS_PWM_NB 8
#endif

extern int32_t actuators_pwm_values[ACTUATORS_PWM_NB];
extern void actuators_pwm_arch_init(void);
extern void actuators_pwm_commit(void);

void actuators_pwm_stop(void);
void actuators_pwm_start(void);
#define SERVOS_TICS_OF_USEC(_v) (_v)

#define ActuatorPwmSet(_i, _v) { actuators_pwm_values[_i] = _v; }
#define ActuatorsPwmCommit  actuators_pwm_commit

inline uint32_t actuators_pwm_get_max_counter(void)
{
//TODO : I think the timers should be sync , so just read one valiable timer enouth

#if PWM_USE_TIM1
  return timer_get_counter(TIM1);
#endif

#if PWM_USE_TIM3
  return timer_get_counter(TIM3);
#endif

#if PWM_USE_TIM4
  return timer_get_counter(TIM4);
#endif

#if PWM_USE_TIM2
  return timer_get_counter(TIM2);
#endif

#if PWM_USE_TIM5
  return timer_get_counter(TIM5);
#endif

#if PWM_USE_TIM8
  return timer_get_counter(TIM8);
#endif
}

#endif /* ACTUATORS_PWM_ARCH_H */
