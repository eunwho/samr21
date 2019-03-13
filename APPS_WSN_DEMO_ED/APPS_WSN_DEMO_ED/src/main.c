/**
* \file  main.c
*/

/************************ HEADERS ****************************************/
#include "asf.h"
#include "sio2host.h"
#include "wsndemo.h"
#include "miwi_api.h"

void rtc_match_callback(void);
void configure_rtc_callbacks(void);
void configure_rtc_calendar(void);
void config_led(void);

struct rtc_module rtc_instance;
struct rtc_calendar_alarm_time alarm;

void rtc_match_callback(void){
	port_pin_toggle_output_level(LED_0_PIN);
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
void ReadMacAddress(void);

struct adc_module adc_instance;

void configure_adc(void)
{
	struct adc_config config_adc;
	
	adc_get_config_defaults(&config_adc);
	adc_init(&adc_instance, ADC, &config_adc);
//	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN6);
	adc_set_positive_input(&adc_instance,ADC_POSITIVE_INPUT_PIN7);
//	adc_set_pin_scan_mode (&adc_instance,ADC_POSITIVE_INPUT_PIN6,2);
	adc_enable(&adc_instance);	
}

void config_led(void){
	struct port_config pin_config;
	port_get_config_defaults( &pin_config);
	pin_config.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &pin_config);
	port_pin_set_output_level(LED_0_PIN, false);
}

uint16_t adcResult;
volatile uint16_t temp[2];

int main ( void )
{
	struct rtc_calendar_time rtc_time;
	
	irq_initialize_vectors();
	system_init();
	delay_init();
	cpu_irq_enable();

	config_led();
/*
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
*/
	sio2host_init();
	wsndemo_init();
	configure_adc();
	
    while(1)
	{
/*
		adc_flush(&adc_instance);
		adc_start_conversion(&adc_instance);
		delay_us(100);
		adc_read(&adc_instance,&adcResult);
		temp[0] = adcResult;
*/
/*		
		port_pin_toggle_output_level(LED_0_PIN);
		delay_ms(500);
*/			
//		adc_start_conversion(&adc_instance);
//		delay_us(100);
//		adc_read(&adc_instance,&adcResult);
//		temp[1] = adcResult;
		wsndemo_task();
    }
}

/*********************************************************************
* Function:         void ReadMacAddress()
*
* PreCondition:     none
*
* Input:		    none
*
* Output:		    Reads MAC Address from MAC Address EEPROM
*
* Side Effects:	    none
*
* Overview:		    Uses the MAC Address from the EEPROM for addressing
*
* Note:			    
**********************************************************************/
void ReadMacAddress(void)
{
}

