/*
 * global.h
 *
 * Created: 2019-03-27 오후 4:21:12
 *  Author: ew2
 */ 


#ifndef _GLOBAL_H_
#define _GLOBAL_H_

struct rtc_module rtc_instance;
struct rtc_calendar_alarm_time alarm;

uint16_t adcResult;
volatile uint16_t temp[2];

struct rtc_calendar_time rtc_time;


#endif /* _GLOBAL_H_ */