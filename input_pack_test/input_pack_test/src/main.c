#define  F_CPU 16000000UL
#include "avr/io.h"
#include <avr/interrupt.h>
#include "util/delay.h"
#include "stdio.h"

#define SET		1
#define RESET	0
#define HIGH	1
#define LOW		0

#define ADDR_485 0x51
//485 RX/TX Enable
//#define RS485_TX_EN PORTE |= _BV(2)
//#define RS485_RX_EN PORTE &= ~_BV(2)

#define RS485_TX_EN ( PORTE |= 0x04  )
#define RS485_RX_EN ( PORTE &= ~0x04 )

unsigned char gState;
unsigned char gflagConnect;

unsigned char gUartRxDone;

// receive protocol {0x02, ADDR_485,'R','3','4','5','6','7','8','9','A', 0x03, 0x00};
// byte 0 STX, 1 address, 2 w don't care, 3 ~ 10 dummy, 11 ETX
 	
unsigned char gUartRxBuffer[14];
unsigned char gUartTxBuffer[14] = {0x02, ADDR_485,'R','3','4','5','6','7','8','9','A', 0x03, 0x00};

volatile unsigned char rx_ch=0;

int sen_num=0;

void usart_init(void);
void port_init(void);

void sendUart(char ch);
void uartPuts(char *str);

ISR(USART0_RX_vect);

long gValue_Current=4321;

int test = 0;
int x = 0;


void byte2Hex(unsigned char inPort, unsigned char * highNible, char * lowNible){
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



// 0x02 0x51 'R' '3456789A' 0x03
int main (void)
{
	unsigned char portIn,highNible,lowNible;
	
	cli();	// 글로벌 인터럽트 디세이블
	port_init();
	usart_init();	
	sei(); // 글로벌 인터럽트 인에이블

	while(1){

		portIn = PINA;
		byte2Hex(portIn, &highNible, &lowNible);
		gUartTxBuffer[3] = highNible;
		gUartTxBuffer[4] = lowNible;

		portIn = PINB;
		byte2Hex(portIn, &highNible, &lowNible);
		gUartTxBuffer[5] = highNible;
		gUartTxBuffer[6] = lowNible;

		portIn = PINC;
		byte2Hex(portIn, &highNible, &lowNible);
		gUartTxBuffer[7] = highNible;
		gUartTxBuffer[8] = lowNible;

		portIn = PIND;
		byte2Hex(portIn, &highNible, &lowNible);
		gUartTxBuffer[9] = highNible;
		gUartTxBuffer[10] = lowNible;
				
		switch(gState){
		case 0 :
			if(gUartRxDone == SET) gState = 1;
			break;			
		case 1 :
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
	}
	return 0;
}
void sendUart(char ch){
	UDR0=ch;
	while(!(UCSR0A&0x40));
	UCSR0A|=0x40;
}

void uartPuts(char *str){
	while(*str)sendUart(*str++);
}


ISR(USART0_RX_vect)
{
	unsigned char inData;

	static unsigned char flagStx = RESET;
	static int gUartIndex=0;
	
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
		if(gUartRxBuffer[1] == ADDR_485 ) gUartRxDone = SET;
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
	DDRA=0x00;
	DDRB=0x00;
	DDRC=0x00;
	DDRD=0x00;
	DDRE=0x06;
	
	PORTA=0x00;
	PORTB=0x00;
	PORTC=0x00;
	PORTD=0x00;
	PORTE=0x00;
	RS485_RX_EN;		// 485수신 대기
}

/*			
		for(i=0;i<8;i++){
			//입력 1번~8번
			if(((PINB>>i)&0x01)!=1)	{
				TX_En;
				TX0_STR("Hello world! DongHo Power Electronics.");
				RX_En;
			} else if(((PIND>>i)&0x01)!=1) {
				TX_En;	TX0_STR("PORTD-ON"); RX_En;
			} else if(((PINC>>i)&0x01)!=1){
				TX_En;	TX0_STR("PORTC-ON"); RX_En;
			} else if(((PINA>>i)&0x01)!=1)	{
				TX_En;	TX0_STR("PORTA-ON"); RX_En;
			}
		}
		_delay_ms(1000);
*/
