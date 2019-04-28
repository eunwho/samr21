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

//---i2cModule.c
extern struct i2c_master_module i2c_master_instance;
extern struct i2c_master_packet i2c_packet;

extern uint8_t	write_buffer[DATA_LENGTH];
extern volatile uint8_t read_buffer[DATA_LENGTH];
extern volatile enum status_code gLcdStatus;
/** UART module for debug. */
extern struct usart_module cdc_uart_module;
volatile extern uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
extern const uint8_t picture1[2100];
extern const uint8_t picture2[2100];
extern const uint8_t vertical[];
extern const uint8_t vertical2[];
extern const uint8_t vertical3[];
extern const uint8_t gLcdRunPic[];
extern const uint8_t gLcdStopPic[];

#endif /* EXTERN_H_ */