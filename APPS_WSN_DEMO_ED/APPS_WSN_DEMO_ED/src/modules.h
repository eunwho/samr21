/*
 * modules.h
 *
 * Created: 2019-03-27 오후 3:59:35
 *  Author: ew2
 */ 


#ifndef _MODULES_H_
#define _MODULES_H_

extern void configure_adc(void);
extern void rtc_match_callback(void);
extern void configure_rtc_callbacks(void);
extern void configure_rtc_calendar(void);
extern void config_led(void);
extern void readMacAddress(void);


extern void UartBytesReceived(uint16_t bytes, uint8_t *byte );

extern void searchConfim(uint8_t foundScanResults, void* ScanResults);
extern void appLinkFailureCallback(void);

extern void wsndemo_init(void);
extern void wsndemo_task(void);

#endif /* _MODULES_H_ */