/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	DDRA=0x00;
	DDRB=0x00;
	DDRC=0x00;
	DDRD=0x00;
	
	PORTA=0xFF;
	PORTB=0xFF;
	PORTC=0xFF;
	PORTD=0xFF;
}
