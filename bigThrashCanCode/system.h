#ifndef _SYSTEM_H_INCLUDED
#define _SYSTEM_H_INCLUDED

#define MATCHTIME 90000UL

#define BLUE 0
#define RED  1

#define JUMPER						((PINA>>PINA7) & 0x01)
#define TAS_SKUPLJAC_LEVI_ZATVOREN	((PINA>>PINA3) & 0x01)
#define TAS_SKUPLJAC_DESNI_ZATVOREN ((PINA>>PINA2) & 0x01)
#define TAS_SKUPLJAC_LEVI_OTVOREN	((PINA>>PINA0) & 0x01)
#define TAS_SKUPLJAC_DESNI_OTVOREN	((PINA>>PINA1) & 0x01)
#define TAS_GOVNO					((PINA>>PINA4) & 0x01)
#define TAS_ROBOT_NAPRED			((PINE>>PINE6) & 0x01)	
#define TAS_ROBOT_NAZAD 			((PINE>>PINE7) & 0x01)	
#define PREK_TAKTIKA1				((PIND>>PIND6) & 0x01)
#define PREK_TAKTIKA2				((PIND>>PIND5) & 0x01)
#define PREK_BOJA					((PIND>>PIND7) & 0x01)

#define DIODA1	0
#define DIODA2	1
#define DIODA3	2
#define DIODA4	3
#define DIODA5	4

#define setPin(port,pin){\
	port|=(1<<pin);\
}\

#define resetPin(port,pin){\
	port&=(~(1<<pin));\
}\

typedef struct{
	volatile unsigned long systemTime;
	volatile char matchStarted;
	volatile char theEnd;
	char boja;
}tSys;

void setRobot(void);
void setColor(void);
unsigned char getColor(void);
unsigned long getSystemTime(void);

void saljiKinezima(unsigned char,unsigned char);

void startMatch(void);
void countTime(void);
void endMatch();

char protocniDelay(unsigned long);
char protocniDelayZwei(unsigned long);
void pauza_ms(unsigned long);

void debaunsingZaTaster(void);

char jumperProvera(void);
char prekBojaProvera(void);
char tasSkupljacDesniOtvoren(void);
char tasSkupljacLeviOtvoren(void);
char tasSkupljacDesniZatvoren(void);
char tasSkupljacLeviZatvoren(void);
char tasGovnoProvera(void);
char tasRobotNazadProvera(void);
char tasRobotNapredProvera(void);
char prekTaktika1Provera(void);
char prekTaktika2Provera(void);


#endif