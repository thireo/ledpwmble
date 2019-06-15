/*
 * ledpwm.c
 *
 * Created: 15-06-2019 19:15:59
 *  Author: Andreas
 */ 
#include "ledpwm.h"

#define PWM_MODULE TCC0
#define PWM_OUT_PIN PIN_PA12F_TCC0_WO6
#define PWM_OUT_MUX PINMUX_PA12F_TCC0_WO6

struct tc_module tc_instance;
void configure_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_COUNTER_SIZE_8BIT;
	config_tc.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc.counter_8_bit.compare_capture_channel[0] = 255;
	config_tc.pwm_channel[0].enabled = true;
	config_tc.pwm_channel[0].pin_out = PWM_OUT_PIN;
	config_tc.pwm_channel[0].pin_mux = PWM_OUT_MUX;
	tc_init(&tc_instance, PWM_MODULE, &config_tc);
	tc_enable(&tc_instance);
}