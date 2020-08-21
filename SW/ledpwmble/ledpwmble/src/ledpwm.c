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

//struct tc_module tc_instance;
/*void configure_tc(void)
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
}*/struct tcc_module tcc_instance;void configure_tcc(void){	struct tcc_config config_tcc;	tcc_get_config_defaults(&config_tcc,PWM_MODULE);	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV256;	//config_tcc.counter.clock_source = GCLK_CLKCTRL_GEN_GCLK0;	config_tcc.counter.period = 0xFFFF;	config_tcc.compare.match[PIN_PA12F_TCC0_WO6] = 100;//(0xFFFF/4);	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTH;	config_tcc.pins.enable_wave_out_pin[PIN_PA12F_TCC0_WO6] = true;	config_tcc.pins.wave_out_pin[PIN_PA12F_TCC0_WO6] = PIN_PA12;	config_tcc.pins.wave_out_pin_mux[PIN_PA12F_TCC0_WO6] = PINMUX_PA12F_TCC0_WO6;		tcc_init(&tcc_instance,PWM_MODULE,&config_tcc);	tcc_enable(&tcc_instance);}void led_pwm_init(void){	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.input_pull = PORT_PIN_PULL_DOWN;
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_PWM_0_PIN,&config_port_pin);	port_pin_set_config(LED_PWM_1_PIN,&config_port_pin);	port_pin_set_config(LED_PWM_2_PIN,&config_port_pin);		port_pin_set_output_level(LED_PWM_2_PIN,false);	//RED	port_pin_set_output_level(LED_PWM_1_PIN,false);	//GREEN	port_pin_set_output_level(LED_PWM_0_PIN,false);	//BLUE}void init_TC3(void)
{	
	PM->APBCMASK.reg |= PM_APBCMASK_TC3;

	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0) | GCLK_CLKCTRL_ID_TCC2_TC3;
	
	TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
	TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_RESYNC;

	TC3->COUNT16.COUNT.reg = 0;
	TC3->COUNT16.CC[0].reg = COMPARE_MATCH;
	
	
	TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	TC3->COUNT16.DBGCTRL.reg = TC_DBGCTRL_DBGRUN;
	TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
	NVIC_EnableIRQ(TC3_IRQn);
	NVIC_SetPriority(TC3_IRQn,2);
	/*PORT->Group[0].DIRSET.reg=18;
	PORT->Group[0].PINCFG[18].bit.PMUXEN=1;
	PORT->Group[0].PMUX[9].bit.PMUXE = 4;*/
}

void TC3_Handler()
{

	static int pwm_count = 0;
	static int dutycycle = PWM_FREQ/8;
	static bool fade_way = true;
	
	pwm_count++;
	if (pwm_count > PWM_FREQ)
	{
		pwm_count = 0;
	}
	
	
		
	
	if (pwm_count > dutycycle)
	{
		port_pin_set_output_level(LED_PWM_0_PIN,true);
		port_pin_set_output_level(LED_PWM_1_PIN,true);
		port_pin_set_output_level(LED_PWM_2_PIN,true);
	}
	else
	{
		port_pin_set_output_level(LED_PWM_0_PIN,false);
		port_pin_set_output_level(LED_PWM_1_PIN,false);
		port_pin_set_output_level(LED_PWM_2_PIN,false);
	}
	if (pwm_count >= PWM_FREQ)
	{	
		if (fade_way)
		{
			dutycycle += 1;
			if(dutycycle >= PWM_FREQ)
			{
				dutycycle = PWM_FREQ;
				fade_way = !fade_way;
			}
		}
		else
		{
			dutycycle -= 1;
			if(dutycycle <= PWM_FREQ/16)
			{
				dutycycle = PWM_FREQ/16;
				fade_way = !fade_way;
			}
		}
		
	}

}