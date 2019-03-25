#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "SolOut.h"

#define SET		1
#define RESET	0
#define HIGH	1
#define LOW		0
#define ADDR_485 0x52
#define RS485_TX_EN ( PORTE |= 0x04  )
#define RS485_RX_EN ( PORTE &= ~0x04 )

unsigned char gState;
unsigned char gflagConnect;

unsigned char gUartRxDone;

unsigned char gUartRxBuffer[14];
unsigned char gUartRxCmd[14];


void usart_init(void);
void port_init(void);

void sendUart(unsigned char ch);
void uartCmdProc(unsigned char * rxCmd);

void uartPuts(unsigned char *str);
void byte2Hex(unsigned char inPort, unsigned char * highNible, unsigned char * lowNible);
unsigned char hex2Int(unsigned char MSB_Byte, unsigned char LSB_Byte);
void readPortState(void);

void byte2Hex(unsigned char inPort, unsigned char * highNible, unsigned char * lowNible){
	unsigned char tmp;
	
	tmp = (( inPort >> 4) & 0x0F );
	if( tmp < 10 ) tmp += 0x30;
	else           tmp += (0x41-10);
	* highNible = tmp;
	
	tmp = (inPort & 0x0F);
	if( tmp < 10 ) tmp += 0x30;
	else           tmp += (0x41 -10);
	* lowNible = tmp;
}


ISR(USART0_RX_vect);

// 0x02, ADDR_485, CMD, DATA[8], 0x03
// CMD == 'R' READ 4 PORT all
// CMD == 'W' WRITE DATA ='0'+'1' ~ '3'+'2'
//    gUartRxCmd = {0x02, 0x52,'W','P','A','5','A','X','X','X','X', 0x03};
// CMD == 'A' write all port with data
//    gUartRxCmd = {0x02, 0x52,'A','F','F','0','0','5','5','A','A', 0x03};


unsigned char gUartTxBuffer[14] = {0x02, ADDR_485,'R','3','4','5','6','7','8','9','A', 0x03, 0x00};

unsigned char hex2Int(unsigned char MSB_Byte, unsigned char LSB_Byte){
	unsigned char tmp1,tmp2;

	tmp1 = MSB_Byte - '0';
	if(tmp1 > 9) tmp1 = MSB_Byte -'A' + 10;
	tmp1 = ( tmp1 << 4 ) & 0xF0;

	tmp2 = LSB_Byte - '0';
	if(tmp2 > 9) tmp2 = LSB_Byte -'A' + 10;
	tmp2 = (tmp2 & 0x0F);
		
	return tmp1+tmp2;
}

void uartCmdProc(unsigned char * rxCmd){

	unsigned char * cmd;
	unsigned char MSB_Byte,LSB_Byte, tmp1,portSel;
	
	cmd = rxCmd;
	
	if(*(cmd+2) == 'A'){		
		MSB_Byte = *(cmd + 3);
		LSB_Byte = *(cmd + 4);
		// MSB_Byte = '0';
		// LSB_Byte = '0';
		PORTA = hex2Int( MSB_Byte, LSB_Byte);

		MSB_Byte = *(cmd + 5);
		LSB_Byte = *(cmd + 6);
		PORTB = hex2Int( MSB_Byte, LSB_Byte);

		MSB_Byte = *(cmd + 7);
		LSB_Byte = *(cmd + 8);
		PORTC = hex2Int( MSB_Byte, LSB_Byte);
		
		MSB_Byte = *(cmd + 9);
		LSB_Byte = *(cmd + 10);
		PORTD = hex2Int( MSB_Byte, LSB_Byte);		 
	} else if(*(cmd+2) == 'W'){

		MSB_Byte = *(cmd + 5);
		LSB_Byte = *(cmd + 6);
		tmp1 = hex2Int( MSB_Byte, LSB_Byte);		
		portSel = *(cmd + 4);
		
		switch( portSel){
		case 'A':
			PORTA = tmp1;
			break;
		case 'B':
			PORTA = tmp1;
			break;
		case 'C':
			PORTA = tmp1;
			break;
		case 'D':
			PORTA = tmp1;
			break;
		}
	}
}
//
void readPortState(void){
	
	unsigned char portIn,highNible,lowNible;
	
	portIn = PORTA;
	byte2Hex(portIn, &highNible, &lowNible);
	gUartTxBuffer[3] = highNible;
	gUartTxBuffer[4] = lowNible;

	portIn = PORTB;
	byte2Hex(portIn, &highNible, &lowNible);
	gUartTxBuffer[5] = highNible;
	gUartTxBuffer[6] = lowNible;

	portIn = PORTC;
	byte2Hex(portIn, &highNible, &lowNible);
	gUartTxBuffer[7] = highNible;
	gUartTxBuffer[8] = lowNible;

	portIn = PORTD;
	byte2Hex(portIn, &highNible, &lowNible);
	gUartTxBuffer[9] = highNible;
	gUartTxBuffer[10] = lowNible;
		
	gUartTxBuffer[1] = gUartRxCmd[1];
	gUartTxBuffer[2] = gUartRxCmd[2];
}

int main (void)
{
	cli();	// 글로벌 인터럽트 디세이블
	port_init();
	usart_init();
	sei(); // 글로벌 인터럽트 인에이블

	while(1){

		switch(gState){
			case 0 :
			if(gUartRxDone == SET) gState = 1;
			break;
		case 1 :
			uartCmdProc(gUartRxCmd);			
			readPortState();
			_delay_ms(2);
			RS485_TX_EN;
			_delay_us(50);
			uartPuts(gUartTxBuffer);
			_delay_us(50);
			RS485_RX_EN;
			gUartRxDone = RESET;
			gState = 0;
			break;
		}
		readPortState();
	}
	return 0;
}

void sendUart(unsigned char ch){
	UDR0=ch;
	while(!(UCSR0A&0x40));
	UCSR0A|=0x40;
}

void uartPuts(unsigned char *str){
	while(*str)sendUart(*str++);
}


ISR(USART0_RX_vect)
{
	unsigned char inData;
	static unsigned char flagStx = RESET;
	static int gUartIndex=0;
	int i ;
	
	inData = UDR0;

	if(flagStx == RESET){
		if(inData == 0x02){
			gUartIndex = 0;
			gUartRxBuffer[gUartIndex++] = inData;
			flagStx = SET;
		}
	} else {
		gUartRxBuffer[gUartIndex++] = inData;		
		if(inData == 0x03){
			gflagConnect = SET;
			flagStx = RESET;
		}else{
			if( gUartIndex >= 14){
				flagStx = RESET;
				gflagConnect = RESET;				
			}
		}
	}

	if(gflagConnect == SET){
		if(gUartRxBuffer[1] == ADDR_485 ){
			gUartRxDone = SET;
			for ( i = 0 ; i < 14 ; i++ ) gUartRxCmd[i] = gUartRxBuffer[i];
		}
		else flagStx = RESET;			
		gflagConnect = RESET;
	}
}

void usart_init(void)
{
	UCSR0A=0;
	UCSR0B=0x98;
	UCSR0C=6;
	UBRR0H=0;
	UBRR0L=103;	// 보오우레이트 9600 설정
}

void port_init(void)
{
	DDRA=0xFF;
	DDRB=0xFF;
	DDRC=0xFF;
	DDRD=0xFF;
	DDRE=0x06;
	
	PORTA=0xFF;
	PORTB=0xFF;
	PORTC=0xFF;
	PORTD=0xFF;
	RX_En; // RS485 수신 대기
}