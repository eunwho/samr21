/*
 * global.h
 *
 * Created: 2019-04-21 오후 6:41:17
 *  Author: ew2
 */ 


#ifndef __GLOBAL_H_
#define __GLOBAL_H_

uint8_t zbeeSensState[10]={0};
uint8_t write_plc[17]={0x00,0x00,0x00,0x00,0x00,0x0B,0x01,0x0F,0x00,0x00,
	0x00,0x20,0x04,0x00,0x00,0x00,0x00};

// rs485
struct usart_module usart_instance;

uint8_t bufTest1[]={0x02,0x52,'A','1','2','3','4','5','6','7','8',0x03,0x00};
uint8_t bufTest2[]={0x02,0x52,'A','0','0','0','0','0','0','0','0',0x03,0x00};
uint8_t bufDout0[]={0x02,0x52,'A','0','0','0','0','0','0','0','0',0x03,0x00};

uint8_t sensStateTable[10] = {0};
	
#endif /* GLOBAL_H_ */