/*
 * SolOut.h
 *
 * Created: 2019-03-20 오후 2:09:47
 *  Author: lee
 */ 


#ifndef SOLOUT_H_
#define SOLOUT_H_


//485 RX/TX Enable
#define TX_En PORTE |= _BV(2)	// RS485 데이터 전송
#define RX_En PORTE &= ~_BV(2)  // RS485 수신대기

// SOL-1A (1번 실린더 전진)
#define sol1A_On  PORTB |= _BV(0)
#define sol1A_Off PORTB &= ~_BV(0)
// SOL-1B (1번 실린더 후진)
#define sol1B_On  PORTB |= _BV(1)
#define sol1B_Off PORTB &= ~_BV(1)

// SOL-2A (2번 실린더 전진)
#define sol2A_On  PORTB |= _BV(2)
#define sol2A_Off PORTB &= ~_BV(2)
// SOL-2B (2번 실린더 후진)
#define sol2B_On  PORTB |= _BV(3)
#define sol2B_Off PORTB &= ~_BV(3)

// SOL-3A (3번 실린더 전진)
#define sol3A_On  PORTB |= _BV(4)
#define sol3A_Off PORTB &= ~_BV(4)
// SOL-3B (3번 실린더 후진)
#define sol3B_On  PORTB |= _BV(5)
#define sol3B_Off PORTB &= ~_BV(5)

// SOL-4A (4번 실린더 전진)
#define sol4A_On  PORTB |= _BV(6)
#define sol4A_Off PORTB &= ~_BV(6)
// SOL-4B (4번 실린더 후진)
#define sol4B_On  PORTB |= _BV(7)
#define sol4B_Off PORTB &= ~_BV(7)

// SOL-5A (5번 실린더 전진)
#define sol5A_On  PORTD |= _BV(0)
#define sol5A_Off PORTD &= ~_BV(0)
// SOL-5B (5번 실린더 후진)
#define sol5B_On  PORTD |= _BV(1)
#define sol5B_Off PORTD &= ~_BV(1)

// SOL-6A (6번 실린더 전진)
#define sol6A_On  PORTD |= _BV(2)
#define sol6A_Off PORTD &= ~_BV(2)
// SOL-6B (6번 실린더 후진)
#define sol6B_On  PORTD |= _BV(3)
#define sol6B_Off PORTD &= ~_BV(3)

// SOL-7A (7번 실린더 전진)
#define sol7A_On  PORTD |= _BV(4)
#define sol7A_Off PORTD &= ~_BV(4)
// SOL-7B (7번 실린더 후진)
#define sol7B_On  PORTD |= _BV(5)
#define sol7B_Off PORTD &= ~_BV(5)

// SOL-8A (8번 실린더 전진)
#define sol8A_On  PORTD |= _BV(6)
#define sol8A_Off PORTD &= ~_BV(6)
// SOL-8B (8번 실린더 후진)
#define sol8B_On  PORTD |= _BV(7)
#define sol8B_Off PORTD &= ~_BV(7)

// SOL-9A (9번 실린더 전진)
#define sol9A_On  PORTC |= _BV(0)
#define sol9A_Off PORTC &= ~_BV(0)
// SOL-9B (9번 실린더 후진)
#define sol9B_On  PORTC |= _BV(1)
#define sol9B_Off PORTC &= ~_BV(1)

// SOL-10A (10번 실린더 전진)
#define sol10A_On  PORTC |= _BV(2)
#define sol10A_Off PORTC &= ~_BV(2)
// SOL-10B (10번 실린더 후진)
#define sol10B_On  PORTC |= _BV(3)
#define sol10B_Off PORTC &= ~_BV(3)

// SOL-11A (11번 실린더 전진)
#define sol11A_On  PORTC |= _BV(4)
#define sol11A_Off PORTC &= ~_BV(4)
// SOL-11B (11번 실린더 후진)
#define sol11B_On  PORTC |= _BV(5)
#define sol11B_Off PORTC &= ~_BV(5)

// SOL-12A (12번 실린더 전진)
#define sol12A_On  PORTC |= _BV(6)
#define sol12A_Off PORTC &= ~_BV(6)
// SOL-12B (12번 실린더 후진)
#define sol12B_On  PORTC |= _BV(7)
#define sol12B_Off PORTC &= ~_BV(7)

// SOL-13A (13번 실린더 전진)
#define sol13A_On  PORTA |= _BV(0)
#define sol13A_Off PORTA &= ~_BV(0)
// SOL-13B (13번 실린더 후진)
#define sol13B_On  PORTA |= _BV(1)
#define sol13B_Off PORTA &= ~_BV(1)

// SOL-14A (14번 실린더 전진)
#define sol14A_On  PORTA |= _BV(2)
#define sol14A_Off PORTA &= ~_BV(2)
// SOL-14B (14번 실린더 후진)
#define sol14B_On  PORTA |= _BV(3)
#define sol14B_Off PORTA &= ~_BV(3)

// SOL-15A (15번 실린더 전진)
#define sol15A_On  PORTA |= _BV(4)
#define sol15A_Off PORTA &= ~_BV(4)
// SOL-15B (15번 실린더 후진)
#define sol15B_On  PORTA |= _BV(5)
#define sol15B_Off PORTA &= ~_BV(5)

// SOL-16A (16번 실린더 전진)
#define sol16A_On  PORTA |= _BV(6)
#define sol16A_Off PORTA &= ~_BV(6)
// SOL-16B (16번 실린더 후진)
#define sol16B_On  PORTA |= _BV(7)
#define sol16B_Off PORTA &= ~_BV(7)

#endif /* SOLOUT_H_ */