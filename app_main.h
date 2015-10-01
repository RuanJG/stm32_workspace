#ifndef __RUAN_APP_MAIN_HEAD
#define __RUAN_APP_MAIN_HEAD

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <mavlink.h>


#define DEBUG_APP 1
#define log(format, ...) if( DEBUG_APP ) printf(format, ## __VA_ARGS__)

#define RADIO_USART_ID usart2_id
#define COPTER_USART_ID usart1_id
#define SBUS_USART_ID usart3_id

#endif
