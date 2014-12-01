#include <avr/io.h>
#include "timers.h"
#include "usart.h"
#include "adc.h"
#include "system.h"

static tSys robot;

void saljiKinezima(unsigned char motor,unsigned char value)
{
	UART1_Write(motor);
	UART1_Write(value);
}

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

static volatile unsigned char i=0;
static volatile unsigned char JUMPER_niz[3];
static volatile unsigned char TAS_SKUPLJAC_LEVI_ZATVOREN_niz[3];
static volatile unsigned char TAS_SKUPLJAC_LEVI_OTVOREN_niz[3];
static volatile unsigned char TAS_SKUPLJAC_DESNI_ZATVOREN_niz[3];
static volatile unsigned char TAS_SKUPLJAC_DESNI_OTVOREN_niz[3];
static volatile unsigned char TAS_GOVNO_niz[3];
static volatile unsigned char TAS_ROBOT_NAPRED_niz[3];
static volatile unsigned char TAS_ROBOT_NAZAD_niz[3];
static volatile unsigned char PREK_TAKTIKA1_niz[3];
static volatile unsigned char PREK_TAKTIKA2_niz[3];
static volatile unsigned char PREK_BOJA_niz[3];

void debaunsingZaTaster(void)
{
	if(++i==3)	i = 0;
	
	JUMPER_niz[i] = JUMPER;
	TAS_SKUPLJAC_LEVI_ZATVOREN_niz[i] = TAS_SKUPLJAC_LEVI_ZATVOREN;
	TAS_SKUPLJAC_LEVI_OTVOREN_niz[i] = TAS_SKUPLJAC_LEVI_OTVOREN;
	TAS_SKUPLJAC_DESNI_ZATVOREN_niz[i] = TAS_SKUPLJAC_DESNI_ZATVOREN;
	TAS_SKUPLJAC_DESNI_OTVOREN_niz[i] = TAS_SKUPLJAC_DESNI_OTVOREN;	
	TAS_GOVNO_niz[i] = TAS_GOVNO;
	TAS_ROBOT_NAZAD_niz[i] = TAS_ROBOT_NAZAD;
	TAS_ROBOT_NAPRED_niz[i] = TAS_ROBOT_NAPRED;
	PREK_TAKTIKA1_niz[i] = PREK_TAKTIKA1;
	PREK_TAKTIKA2_niz[i] = PREK_TAKTIKA2;
	PREK_BOJA_niz[i] = PREK_BOJA;
}


char prekTaktika1Provera(void)
{
	if(PREK_TAKTIKA1_niz[0] | PREK_TAKTIKA1_niz[1] | PREK_TAKTIKA1_niz[2])
		return 0;
		
	return 1;	
}

char prekTaktika2Provera(void)
{
	if(PREK_TAKTIKA2_niz[0] | PREK_TAKTIKA2_niz[1] | PREK_TAKTIKA2_niz[2])
		return 0;
		
	return 1;	
}

char jumperProvera(void)
{
	if(JUMPER_niz[0] | JUMPER_niz[1] | JUMPER_niz[2])
		return 0;
		
	return 1;
}

char prekBojaProvera(void)
{
	if(PREK_BOJA_niz[0] | PREK_BOJA_niz[1] | PREK_BOJA_niz[2])
		return 0;
		
	return 1;
}

char tasSkupljacDesniZatvoren(void)
{
	if(TAS_SKUPLJAC_DESNI_ZATVOREN_niz[0] | TAS_SKUPLJAC_DESNI_ZATVOREN_niz[1] | TAS_SKUPLJAC_DESNI_ZATVOREN_niz[2])
		return 0;
		
	return 1;
}

char tasSkupljacLeviZatvoren(void)
{
	if(TAS_SKUPLJAC_LEVI_ZATVOREN_niz[0] | TAS_SKUPLJAC_LEVI_ZATVOREN_niz[1] | TAS_SKUPLJAC_LEVI_ZATVOREN_niz[2])
		return 0;
		
	return 1;
}

char tasSkupljacLeviOtvoren(void)
{
	if(TAS_SKUPLJAC_LEVI_OTVOREN_niz[0] | TAS_SKUPLJAC_LEVI_OTVOREN_niz[1] | TAS_SKUPLJAC_LEVI_OTVOREN_niz[2])
		return 0;
		
	return 1;
}

char tasSkupljacDesniOtvoren(void)
{
	if(TAS_SKUPLJAC_DESNI_OTVOREN_niz[0] | TAS_SKUPLJAC_DESNI_OTVOREN_niz[1] | TAS_SKUPLJAC_DESNI_OTVOREN_niz[2])
		return 0;
		
	return 1;
}

char tasGovnoProvera(void)
{
	if(TAS_GOVNO_niz[0] | TAS_GOVNO_niz[1] | TAS_GOVNO_niz[2])
		return 0;
		
	return 1;	
}

char tasRobotNapredProvera(void)
{
	if(TAS_ROBOT_NAPRED_niz[0] | TAS_ROBOT_NAPRED_niz[1] | TAS_ROBOT_NAPRED_niz[2])
		return 0;
		
	return 1;	
}

char tasRobotNazadProvera(void)
{
	if(TAS_ROBOT_NAZAD_niz[0] | TAS_ROBOT_NAZAD_niz[1] | TAS_ROBOT_NAZAD_niz[2])
		return 0;
		
	return 1;	
}

void startMatch(void)
{
	robot.systemTime = 0;
	robot.theEnd = 0;
	robot.matchStarted = 1;
}

void endMatch(void)
{
	altUart1Write('A');
	altUart1Write(0);
	altUart1Write('B');
	altUart1Write(0);
	altUart1Write('C');
	altUart1Write(0);
	altUart1Write('D');
	altUart1Write(0);
	altUart0Write('S');
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
	initUart0();
	initUart1();
	
	timer1Init();
	timer3Init();
	
	adcInit();
	
	DDRA=0x00;
	PORTA=0xFF;
	
	DDRE=0x00;
	PORTE=0xFF;
	
	DDRD=0x00;
	PORTD=0xFF;
	
	DDRG=0xFF;
	PORTG=0x00;
	
	pauza_ms(1000);
	
	setColor();
}

void setColor(void)
{
	if(prekBojaProvera())
		robot.boja=BLUE;
	else 
		robot.boja=RED;
}

unsigned char getColor(void)
{
	return robot.boja;
}

unsigned long getSystemTime(void)
{
	return robot.systemTime;
}