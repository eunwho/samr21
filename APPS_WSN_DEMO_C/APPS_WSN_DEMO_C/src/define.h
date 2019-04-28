/*
 * define.h
 *
 * Created: 2019-04-22 오후 5:56:47
 *  Author: ew2
 */ 
#ifndef DEFINE_H_
#define DEFINE_H_

#define MAX_RX_BUFFER_LENGTH   14
#define  txd_en PORT->Group[0].OUTSET.reg=PORT_PA27
#define  rxd_en PORT->Group[0].OUTCLR.reg=PORT_PA27

//--- i2cMaster.c
#define DATA_LENGTH			10
#define TIMEOUT				1000

//#0define ADDR_AT24C02D		0xA0
//#define ADDR_AT24C02D		0x50
#define ADDR_GLCD			0x3F
#define MAX_RX_BUFFER_LENGTH   14
#define GLCD_COMMAND	0x00
#endif /* DEFINE_H_ */