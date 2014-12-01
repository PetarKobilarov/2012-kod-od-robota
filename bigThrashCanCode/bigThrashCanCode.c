/***************************************************************************************************************************************************************
*															ELECTROPIONEER 2012
****************************************************************************************************************************************************************/													
#define F_CPU 11059200UL
#include "system.h"
#include "kretanje.h"
#include "funkcije.h"
#include "lcd.h"

int main(void)
{
	//inicijalizacija robota
	setRobot();

	InitLCD(0);
	LCDClear();
	
	LCDWriteStringXY(0,1,"STRANA: ")  
	
	if(getColor() == BLUE)
	{
		LCDWriteStringXY(8,1,"PLAVA");
	}
	else
	{
		LCDWriteStringXY(8,1,"CRVENA");
	}	
			
	LCDWriteStringXY(0,0,"Taktika:");
	
	LCDWriteStringXY(0,0,"Taktika:");
	
	if(prekTaktika1Provera() && prekTaktika2Provera())
	{
		LCDWriteStringXY(8,0,"T1T2MK");
	}else if((!prekTaktika1Provera()) && (!prekTaktika2Provera()))
	{
		LCDWriteStringXY(8,0,"T1T2MT3K");
	}else if(prekTaktika1Provera())
	{
		LCDWriteStringXY(8,0,"T1T2T3MK");	
	}else
		LCDWriteStringXY(8,0,"T1T2KMK");
	
	while(jumperProvera())
	{
		//za pustanje mape pritiskom na taster posle meca
		if(tasGovnoProvera())
		{
			saljiKinezima(MOTOR_SKUPLJAC_MAPE,50);
			while(tasGovnoProvera());
			saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);	
		}	
	}
	
	//pocinje brojanje vremena
	startMatch();
	
	while(1)
	{
	
		if(getColor() == BLUE)
			blueSide();
		else
			redSide();	
		
	}
	
	return 0;
}