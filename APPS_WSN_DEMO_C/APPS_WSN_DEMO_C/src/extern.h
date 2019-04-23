/*
 * extern.h
 *
 * Created: 2019-04-22 오후 5:58:46
 *  Author: ew2
 */ 


#ifndef __EXTERN_H_
#define __EXTERN_H_

extern uint8_t zbeeSensState[10];
extern uint8_t write_plc[17];
extern struct usart_module usart_instance;

extern uint8_t bufTest1[13];
extern uint8_t bufTest2[13];
extern uint8_t sensStateTable[10];

#endif /* EXTERN_H_ */