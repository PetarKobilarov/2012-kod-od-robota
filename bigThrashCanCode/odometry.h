#ifndef _ODOMETRY_H_INCLUDED
#define _ODOMETRY_H_INCLUDED

#define PUNKRUG  57020L   //treba uneti vrednost koja je na robotu

#define NORMALPRECISION  5
#define UPPRECISION		25        
#define NORMALSPEED		2500     //2500  
#define UPSPEED			3000
#define LOWSPEED		1000

#define PI 3.141592654


void Stop(void);
void setCommandInt(unsigned char,int);
void setCommandShort(unsigned char,short);
int readX(void);
int readY(void);
long readUgaoInkrementi(void);
int readUgaoStepeni(void);
int readDistanca(void);
void setStartPosition(int,int,int);
int absolut(int);
int racunanjeUgla(int, int);
void setUgaoNula(void);
char setUgao(int);
//char moveOnDirectionReal(int,short,int,char,int);
char gotoXY(int,int,signed char,int,int,char);
char moveOnDirection(int,short,int,char,int);

//*******************************OBAVEZNO!!!!****************************************************************************************///
//PROJECT PROPERTIES->TOOLCHAIN->AVR/GNU C LINKER->LIBRARIES->GORNJI DEO DODATI libm.a(ovo govno je zabagovanije od sugavog codevisiona)
//*******************************OBAVEZNO!!!!****************************************************************************************///

#endif