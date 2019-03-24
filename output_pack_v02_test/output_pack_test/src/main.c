#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "SolOut.h"

void usart_init(void);
void port_init(void);

void TX0_CH(char ch);
void TX0_STR(char *str);

ISR(USART0_RX_vect);

volatile unsigned char rx_ch;

int main (void)
{
	char gUartTxBuffer[9] ={0x02,0x50,'A','1','0','0','0',0x30};
	cli();	// 글로벌 인터럽트 디세이블
	
	/* Insert system clock initialization code here (sysclk_init()). */	
	port_init();
	usart_init();

	
//	sei(); // 글로벌 인터럽트 인에이블

	/* Insert application code here, after the board has been initialized. */
	while(1)
	{		
/*		
		sol1A_On;
		_delay_ms(250);
		sol1A_Off;
		_delay_ms(250);
*/		

		PORTA = 0;
		PORTB = 0;
		PORTC = 0;
		PORTD = 0;
		_delay_ms(2000);
		PORTA = 0xFF;
		PORTB = 0xFF;
		PORTC = 0xFF;
		PORTD = 0xFF;
		_delay_ms(2000);

		TX_En;
		_delay_ms(10);		
		TX0_STR(gUartTxBuffer);
		RX_En;
				
	}
}

// 송신 1바이트
void TX0_CH(char ch)
{
//	TX_En;
	UDR0=ch;
	while(!(UCSR0A&0x40));
	UCSR0A|=0x40;
//	RX_En;
}

// 송신 문자열
void TX0_STR(char *str)
{
	while(*str)TX0_CH(*str++);
}

// 수신 인터럽트
ISR(USART0_RX_vect)
{
	rx_ch=UDR0;
	
	if(rx_ch=='a')
	{
		sol1B_On;
		TX0_CH(rx_ch);
	}
	else if(rx_ch=='b')
	{
		sol1B_Off;
		TX0_CH(rx_ch);
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