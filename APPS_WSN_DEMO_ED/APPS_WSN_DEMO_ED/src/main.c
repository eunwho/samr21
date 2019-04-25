/**
* \file  main.c
*/
#include "asf.h"
#include "sio2host.h"
#include "wsndemo.h"
#include "miwi_api.h"
#include "config.h"

struct rtc_module rtc_instance;

void rtc_overflow_callback(void);
void configure_rtc_count(void);
void configure_rtc_callbacks(void);

void rtc_overflow_callback(void){	
	//port_pin_set_output_level(PIN_PA28,true);
	//delay_us(20);
	port_pin_toggle_output_level(PIN_PA28);
	//port_pin_set_output_level(PIN_PA28,false);
}

void config_led(void);
void readMacAddress(void);

AppState_t appState = APP_STATE_INITIAL;

int temp1;

void configure_rtc_count(void){
	
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);
	
	config_rtc_count.prescaler				= RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode					= RTC_COUNT_MODE_16BIT;
	config_rtc_count.continuously_update	= true;

	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	rtc_count_enable(&rtc_instance);
}

void configure_rtc_callbacks(void){
	rtc_count_register_callback( &rtc_instance, rtc_overflow_callback,RTC_COUNT_CALLBACK_OVERFLOW);
	rtc_count_enable_callback( &rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW);
}

void configure_adc(void);
//void ReadMacAddress(void);

struct adc_module adc_instance;

void configure_adc(void)
{
	struct adc_config config_adc;
	
	adc_get_config_defaults(&config_adc);
	adc_init(&adc_instance, ADC, &config_adc);
	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN6);
	adc_enable(&adc_instance);	
}

void config_led(void){
	struct port_config pin_config;
	port_get_config_defaults( &pin_config);
	pin_config.direction  = PORT_PIN_DIR_OUTPUT;
//	port_pin_set_config(LED_0_PIN, &pin_config);
//	port_pin_set_output_level(LED_0_PIN, false);
	port_pin_set_config(PIN_PA27, &pin_config);
	port_pin_set_output_level(PIN_PA27, false);

	port_get_config_defaults( &pin_config);
	pin_config.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA28, &pin_config);
	port_pin_set_output_level(PIN_PA28, false);
}


uint16_t adcResult;
volatile uint16_t tmp1, tmp2;

void readMacAddress(void){
	myLongAddress[0] = 16;
	myLongAddress[1] = 53;
	myLongAddress[2] = 0;
	myLongAddress[3] = 32;
	myLongAddress[4] = 89;
	myLongAddress[5] = 37;
	myLongAddress[6] = 128;
	myLongAddress[7] = MAC_ADDR;	
}

int main ( void )
{
	system_init();
	delay_init();
	irq_initialize_vectors();
	config_led();
	readMacAddress();
	wsndemo_init();
	configure_adc();
	configure_rtc_count();
	configure_rtc_callbacks();
	rtc_count_set_period(&rtc_instance, 800);
	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
	cpu_irq_enable();	
    while(1)
	{
/*
		if(tmp1 > 19 ){
			system_sleep();
			tmp1 = 0 ;
		} else {
			delay_ms(200);
			port_pin_toggle_output_level(PIN_PA28);			
			tmp1++;
		}
*/
		wsndemo_task();
    }
}