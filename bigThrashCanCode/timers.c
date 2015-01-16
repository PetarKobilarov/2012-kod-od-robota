#include <avr/io.h>
#include "timers.h"

void Timer1_Init(void)
{
	TCCR1A=0;
	TCCR1B=(1<<WGM12) | (1<<CS10);
	OCR1AH=0x2B;  //za kvarc od 8Mhz OCR=0x1F40
	OCR1AL=0x33;
	TIMSK=1<<OCIE1A;	
	
	SREG |= 0x80;	
}

void Timer3_Init(void)
{
	TCCR3A=0;
	TCCR3B=(1<<WGM32) | (1<<CS30);
	OCR3AH=0x2B;
	OCR3AL=0x33;
	ETIMSK=1<<OCIE3A;
	
	SREG |= 0x80;
}

ISR(TIMER1_COMPA_vect)
{
	countTime();	
	debaunsingZaTaster();			
}

extern char flagPauze;
extern unsigned long passedTime;
ISR(TIMER3_COMPA_vect)
{
	if(flagPauze)
		++passedTime;		
}