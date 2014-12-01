#ifndef _KRETANJE_H_INCLUDED
#define _KRETANJE_H_INCLUDED

#define NO_BREAK_TIME			   0
#define ONE_SECOND_BREAK_TIME	1200
#define TWO_SECOND_BREAK_TIME	2000

typedef enum{
	TRUE,
	FALSE
}eBool;

typedef enum{
	FAST_AND_FURIOUS,
	RIDE_THE_LIGHTING,
	COLLISION	
}eNesto;

void blueSide(void);
void redSide(void);


#endif
