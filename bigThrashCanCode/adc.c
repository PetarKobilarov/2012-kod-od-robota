#ifndef F_CPU
#define F_CPU 11059200UL

#endif

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "adc.h"

void adcInit(void)
{
	ADCSRA=(1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);
	ADMUX=1<<REFS0;
}

unsigned int adcRead(unsigned char input)
{
	unsigned int temp;

	ADMUX=(1<<REFS0) | input;	
	ADCSRA |= 0x40;
	
	_delay_us(20);
	
	while(!((ADCSRA>>ADIF) & 0x01));
	
	ADCSRA |= (1<<ADIF);
	
	temp=ADCL | (ADCH<<8);
	
	return temp;
}

unsigned int distancaProtivnika(signed char channel)
{
    float adc_vrednost, value;
    float  distanca = 0;
  
    if(channel == (-1))
		value = adcRead(0);
	else
		value = adcRead(7);
	
	distanca = (2914/(value+5))-1.5;
   // adc_vrednost = 4187.8/((float)value); 
    
    //distanca = (unsigned int) (pow(adc_vrednost, 1.1060));

    /*
    adc_vrednost = (float)read_adc(0);
    adc_vrednost = (float)((adc_vrednost/1024.0)*5.0);
    
    distanca = ((1/((0.029*adc_vrednost) + 0.005)));//-4
    distanca =  (long)distanca;
      */
    return distanca;
        
}

char proveriProtivnika(signed char channel,int upValue)
{
    unsigned int adcProtivnik;
	char brojacProtivnika = 0;
    char i;
    
    for(i=0; i<5; i++)
    {
        _delay_ms(20);  //ceka 20ms zbog sharpa
        adcProtivnik = distancaProtivnika(channel);
        if((adcProtivnik>20) && (adcProtivnik<upValue)) //OVO TREBA TESTIRATI DOBRO!!!
            brojacProtivnika++;               
    }
    
    if(brojacProtivnika == 5) 
		return 1;
    
    return 0;   
}