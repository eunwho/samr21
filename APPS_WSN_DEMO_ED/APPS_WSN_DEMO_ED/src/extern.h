/*
 * extern.h
 *
 * Created: 2019-03-27 오후 4:44:27
 *  Author: ew2
 */ 


#ifndef _EXTERN_H_
#define _EXTERN_H_

//--- main.c
extern struct rtc_module rtc_instance;
extern struct rtc_calendar_alarm_time alarm;

extern uint16_t adcResult;
extern volatile uint16_t temp[2];

extern struct adc_module adc_instance;
extern struct rtc_calendar_time rtc_time;

#endif /* _EXTERN_H_ */