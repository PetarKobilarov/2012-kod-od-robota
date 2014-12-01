#ifndef _ADC_H_INCLUDED
#define _ADC_H_INCLUDED

void adcInit(void);
unsigned int adcRead(unsigned char);
unsigned int distancaProtivnika(signed char);
char proveriProtivnika(signed char,int);
#endif