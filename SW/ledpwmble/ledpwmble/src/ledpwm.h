/*
 * ledpwm.h
 *
 * Created: 15-06-2019 19:47:53
 *  Author: Andreas
 */ 


#ifndef LEDPWM_H_
#define LEDPWM_H_

#include "asf.h"

#define GCLK_FREQ 8000000
#define PWM_FREQ 250
#define COMPARE_MATCH (GCLK_FREQ/PWM_FREQ)

#if ((COMPARE_MATCH > 65535))
#error "TOO LARGE COMPARE_MATCH VALUE. TRY INCREASING PWM_FREQ"
#endif

#define LED_PWM_0_PIN PIN_PA12
#define LED_PWM_1_PIN PIN_PA14
#define LED_PWM_2_PIN PIN_PB11

int pwm_count;


void led_pwm_init(void);
void configure_tcc(void);

void init_TC3(void);
void TC3_Handler();

#endif /* LEDPWM_H_ */