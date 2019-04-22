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


#endif /* DEFINE_H_ */