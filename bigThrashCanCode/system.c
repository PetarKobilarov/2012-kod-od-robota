#include <avr/io.h>
#include "timers.h"
#include "usart.h"
#include "adc.h"
#include "system.h"

static tSys robot;


volatile unsigned char counting = 0;
static volatile unsigned long targetTime;

char protocniDelay(unsigned long zeljeno_vreme)
{	
	if(counting == 0)
	{
		counting=1;
		targetTime=zeljeno_vreme+robot.systemTime;
		return 0;
	}	
	
	if(robot.systemTime >= targetTime)
	{
		counting=0;
		return 1;
	}
	
	return 0;	
}

volatile unsigned char countingZwei = 0;
static volatile unsigned long targetTimeZwei;

char protocniDelayZwei(unsigned long zeljeno_vreme)
{	
	if(countingZwei == 0)
	{
		countingZwei = 1;
		targetTimeZwei = zeljeno_vreme + robot.systemTime;
		
		return 0;
	}	
	
	if(robot.systemTime >= targetTimeZwei)
	{
		countingZwei = 0;
		
		return 1;
	}
	
	return 0;	
}

volatile char flagPauze;
volatile unsigned long passedTime;

void pauza_ms(unsigned long zeljeno_vreme)
{
	passedTime=0;
	flagPauze=1;
	
	while(passedTime<=zeljeno_vreme);
	
	flagPauze=0;
}



void startMatch(void)
{
	robot.systemTime = 0;
	robot.theEnd = 0;
	robot.matchStarted = 1;
}



void countTime(void)
{
	if(robot.matchStarted)
	{
		if(++robot.systemTime>=MATCHTIME)
		{
			robot.theEnd=1;		
			endMatch();
			while(1)
				PORTG = 0xFF;
		}
	}
}

void setRobot(void)
{
	
	Timer1_Init();
	Timer3_Init();
	

}

void setColor(void)
{

}

unsigned char getColor(void)
{
	return robot.boja;
}

unsigned long getSystemTime(void)
{
	return robot.systemTime;
}