#ifndef _TIMERS_H_INCLUDED
#define _TIMERS_H_INCLUDED

#include <avr/interrupt.h>

#define activateInterrupts()	sei()	
#define disableInterrupts()		cli()

void Timer1_Init(void);
void Timer3_Init(void);

#endif