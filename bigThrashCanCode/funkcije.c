#include "funkcije.h"
#include "system.h"
#include "lcd.h"
#include "adc.h"

void zatvoriSkupljac(void)
{
	saljiKinezima(MOTOR_SKUPLJAC_LEVI,SKUPLJAC_BRZINA);
	saljiKinezima(MOTOR_SKUPLJAC_DESNI,SKUPLJAC_BRZINA+100);
	
	while((!tasSkupljacDesniZatvoren()) || (!tasSkupljacLeviZatvoren()));
	
	saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
	saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
}

void otvoriSkupljac(void)
{
	saljiKinezima(MOTOR_SKUPLJAC_LEVI,SKUPLJAC_BRZINA+100);
	saljiKinezima(MOTOR_SKUPLJAC_DESNI,SKUPLJAC_BRZINA);
	
	while((!tasSkupljacDesniOtvoren()) || (!tasSkupljacLeviOtvoren()));
	
	saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
	saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
}

void pokupiMapu(void)
{
	saljiKinezima(MOTOR_SKUPLJAC_MAPE,SKUPLJAC_MAPE_BRZINA);
	pauza_ms(3500);
	saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);
}

void skupljacCim(unsigned char motor,unsigned char value)
{
	if(motor == MOTOR_SKUPLJAC_DESNI)
		saljiKinezima(MOTOR_SKUPLJAC_DESNI,value+100);
	else
		saljiKinezima(MOTOR_SKUPLJAC_LEVI,value);
		
	pauza_ms(120);
	
	otvoriSkupljac();
}

extern int readX(void);
extern int readY(void);

void writeCoord(void)
{
	LCDClear();
	
	LCDWriteStringXY(0,0,"X: ");
	LCDWriteStringXY(0,1,"Y: ");
	
	LCDGotoXY(4,0);
	LCDWriteInt(readX(),4);
	
	LCDGotoXY(4,1);
	LCDWriteInt(readY(),4);
}

void writeTime(void)
{
	long time = getSystemTime();
	int sec;
	
	LCDClear();
	
	LCDWriteStringXY(0,0,"Vreme : ");
	
	sec = time/1000;
	
	LCDWriteIntXY(9,0,sec,2);
}


char getDetections(char senzor,signed char side)
{
	if((senzor == ALL_DETECTIONS_ON) || (senzor == TASTER_SUDAR_ON))
	{
		if(side == -1)
			if(tasRobotNazadProvera())
				return 1;
				
		if(side == 1)
			if(tasRobotNapredProvera())
				return 1;
	}	
	
	if((senzor == ALL_DETECTIONS_ON) || (senzor == SHARP_ON))
	{
		if(proveriProtivnika(side,SHARP_NORMAL_DISTANCE))
			return 1;
	}
	
	return 0;				
}