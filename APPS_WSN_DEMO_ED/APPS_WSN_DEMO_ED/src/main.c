/**
* \file  main.c
*/
#include "asf.h"
#include "sio2host.h"
#include "wsndemo.h"
#include "miwi_api.h"
#include "config.h"

void rtc_match_callback(void);
void configure_rtc_callbacks(void);
void configure_rtc_calendar(void);
void config_led(void);
void readMacAddress(void);

struct rtc_module rtc_instance;
struct rtc_calendar_alarm_time alarm;
//extern AppState_t appState;
// extern void appSendData();

int temp1;

void rtc_match_callback(void){
	
	// port_pin_toggle_output_level(LED_0_PIN);
//	LED_Toggle(LED_NETWORK); //by jsk
/*
	if (APP_STATE_WAIT_SEND_TIMER == appState) {
		appState = APP_STATE_SEND;
	}
*/
	alarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;	
	alarm.time.second += 1;
	alarm.time.second = alarm.time.second % 60;
	rtc_calendar_set_alarm(&rtc_instance, &alarm, RTC_CALENDAR_ALARM_0);
}

void configure_rtc_callbacks(void){
	rtc_calendar_register_callback(	&rtc_instance, rtc_match_callback, RTC_CALENDAR_CALLBACK_ALARM_0);
	rtc_calendar_enable_callback(	&rtc_instance, RTC_CALENDAR_CALLBACK_ALARM_0);	
}

void configure_rtc_calendar(void){
	struct rtc_calendar_config config_rtc_calendar;
	rtc_calendar_get_config_defaults(&config_rtc_calendar);
	alarm.time.year		= 2019;
	alarm.time.month	= 3;
	alarm.time.day		= 11;
	alarm.time.hour		= 8;
	alarm.time.minute	= 0;
	alarm.time.second	= 10;
	
	config_rtc_calendar.clock_24h = true;
	config_rtc_calendar.alarm[0].time = alarm.time;
	config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_YEAR;
	
	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);	
	rtc_calendar_enable(&rtc_instance);
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
	port_pin_set_config(LED_0_PIN, &pin_config);
	port_pin_set_output_level(LED_0_PIN, false);

	port_get_config_defaults( &pin_config);
	pin_config.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA28, &pin_config);
	port_pin_set_output_level(PIN_PA28, false);
}


uint16_t adcResult;
volatile uint16_t temp[2];

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
	struct rtc_calendar_time rtc_time;
	
	irq_initialize_vectors();
	system_init();
	delay_init();
	cpu_irq_enable();

	config_led();
	rtc_calendar_get_time_defaults(&rtc_time);
	rtc_time.year	= 2019;
	rtc_time.month	= 3;
	rtc_time.day	= 11;
	rtc_time.hour	= 8;
	rtc_time.minute	= 0;
	rtc_time.second	= 0;

	configure_rtc_calendar();
	configure_rtc_callbacks();
	rtc_calendar_set_time(&rtc_instance, &rtc_time);
	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);

	readMacAddress();
	wsndemo_init();
	configure_adc();
    while(1)
	{
		wsndemo_task();
    }
}