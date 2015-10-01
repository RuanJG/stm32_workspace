#include "delay.h"

static volatile int timer_time =0;
static int timer_inited = 0;

static void tim_setup(void)
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral. */
	timer_reset(TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/* Reset prescaler value. */
	timer_set_prescaler(TIM2, 1000-1);//72MHz/1000=72000hz, ;

	/* Enable preload. */
	timer_disable_preload(TIM2);

	/* Continous mode. */
	timer_continuous_mode(TIM2);

	/* Period (36kHz). */
	//timer_set_period(TIM2, 36);//65535);

	//ms  72khz
	timer_set_period(TIM2, 72);// 72000hz /72 = 1000hz =1ms

	/* Disable outputs. */
	timer_disable_oc_output(TIM2, TIM_OC1);
	timer_disable_oc_output(TIM2, TIM_OC2);
	timer_disable_oc_output(TIM2, TIM_OC3);
	timer_disable_oc_output(TIM2, TIM_OC4);

	/* -- OC1 configuration -- */

	/* Configure global mode of line 1. */
	//timer_disable_oc_clear(TIM2, TIM_OC1);
	//timer_disable_oc_preload(TIM2, TIM_OC1);
	//timer_set_oc_slow_mode(TIM2, TIM_OC1);
	//timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);

	/* Set the capture compare value for OC1. */
	//timer_set_oc_value(TIM2, TIM_OC1, 1000);

	/* ---- */

	/* ARR reload enable. */
	timer_disable_preload(TIM2);

	/* Counter enable. */
	//timer_enable_counter(TIM2);
	/* Enable commutation interrupt. */
	//timer_enable_irq(TIM2, TIM_DIER_CC1IE);

	timer_inited = 1;
	timer_time = 0;
}

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {

		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_CC1IF);

		/*
		 * Get current timer value to calculate next
		 * compare register value.
		 */
		 
		//val = timer_get_counter(TIM2);
		if( timer_time > 0){
			timer_time--;	
		};
		

		/* Calculate and set the next compare value. */
		//timer_set_oc_value(TIM2, TIM_OC1, 2000);
	}
}
void timer_start(int ms)
{
	timer_time = ms;
	timer_disable_preload(TIM2);//set the period to reg
	timer_enable_counter(TIM2);
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}
void timer_stop()
{
	timer_disable_irq(TIM2, TIM_DIER_CC1IE);
	timer_disable_counter(TIM2);
}

void timer_mdelay(int ms)
{

	if( timer_inited == 0)
		tim_setup();
	
	timer_start(ms);
	while(timer_time > 0);
	timer_stop();

}






static volatile int systick_time=0;
static int systick_timer_inited = 0;
void systick_setup()
{
	/* 72MHz / 8 => 9000000 counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	//systick_interrupt_enable();
	//systick_counter_enable();
	systick_interrupt_disable();
	systick_counter_disable();
	systick_timer_inited =1;
}

void sys_tick_handler(void)
{
	systick_time ++;
}
void systick_mdelay(int ms)
{
	if( systick_timer_inited == 0)
		systick_setup();

	if( ms <= 0 )
		return ;

	systick_time = 0;	
	systick_interrupt_enable();
	systick_counter_enable();

	while(systick_time < ms);

	systick_counter_disable();
	systick_interrupt_disable();
	//systick_time = 0;
}


void mdelay(int ms)
{
	systick_mdelay(ms);
}
