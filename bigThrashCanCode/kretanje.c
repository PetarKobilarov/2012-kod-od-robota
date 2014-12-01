#ifndef F_CPU
#define F_CPU 11059200UL
#endif

#include <avr/pgmspace.h>
#include <util/delay.h>
#include "kretanje.h"
#include "system.h"
#include "odometry.h"
#include "funkcije.h"
#include "adc.h"

/*****************************************   PUTANJE ZA PLAVU STRANU********************************************************/
#define brojPozicijaPlavo 9
unsigned int pathXBlue[brojPozicijaPlavo] PROGMEM=
{
	2596,  //dolazak ispred totema  2628
	1720,  //ide na drugi kraj stola, nakon sto izadje iz cilja kada istovari prvu turu
	1300,  //dolazi do drugog dela prvog totema, pre okretanja ka njemu
	2660,  //ide ispred mape, posle izlaska iz cilja - posle istovaranja druge ture
	2600,	//ide paralelno zidu do protivnickog totema  2615
	2700,	//posle kupljenja sa protivnickog totema ide blizu nase kabine
	2700,	//ulazi u start- kabinu
	2700,	//ide na drugi kraj stola
	2125	//dolazi do protivnickog broda	
	
};

unsigned int pathYBlue[brojPozicijaPlavo] PROGMEM=
{	
	2300,	
	1680,	
	2245,
	2425,
	3035,
	1930,
	1380,
	3330, ///3320
	3340		
};

signed char directionBlue[brojPozicijaPlavo] PROGMEM=
{
	1,
   -1,
   -1,
   -1,
    1,
    1,
	1,
   -1,
   -1	 	
};

#define brojPozicijaPlavoAlt 4
unsigned int pathXBlueAlt[brojPozicijaPlavoAlt] PROGMEM=
{
	2740,		//u slucaju sudara na prvom delu putanje do mape, ide do zida
	2740, 		//pokusava uz zid da dodje do mape
	2400,		//dolazi do drveta i pokusava tako do mape
	2400		//pozicija odakle nastavlja sa kupljenjem drugog dela naseg totema
		
};	

unsigned int pathYBlueAlt[brojPozicijaPlavoAlt] PROGMEM=
{
	1650,
	2440,
	2440,
	1680
};

signed char directionBlueAlt[brojPozicijaPlavoAlt] PROGMEM=
{
	 1,
	-1,
	 1,
	 1	 	
}; 


/*****************************************   PUTANJE ZA CRVENU STRANU********************************************************///////////////

#define brojPozicijaCrveno 9
unsigned int pathXRed[brojPozicijaCrveno] PROGMEM=
{
	2596,  //dolazak ispred totema  2628
	1720,  //ide na drugi kraj stola, nakon sto izadje iz cilja kada istovari prvu turu
	1300,  //dolazi do drugog dela prvog totema, pre okretanja ka njemu  1420  MENJANO!
	2660,  //ide ispred mape, posle izlaska iz cilja - posle istovaranja druge ture
	2600,	//ide paralelno zidu do protivnickog totema  2615
	2700,	//posle kupljenja sa protivnickog totema ide blizu nase kabine
	2700,	//ulazi u start- kabinu  2820
	2700,	//ide na drugi kraj stola
	2125	//dolazi do protivnickog broda	
};

unsigned int pathYRed[brojPozicijaCrveno] PROGMEM=
{	
	2700, //2300,	
	3320, //1680,	
	2755, //2095, MENJANO!
	2595, //2425,  HSFUSFYU B  2575
	1965, //3035,
	3070, //1930,
	3620, //1380,
	1670, //3330,
	1660  //3340	
};

signed char directionRed[brojPozicijaCrveno] PROGMEM=
{
	1,
   -1,
   -1,
   -1,
    1,
    1,
	1,
   -1,
   -1	 	
};

#define brojPozicijaCrvenoAlt 4
unsigned int pathXRedAlt[brojPozicijaCrvenoAlt] PROGMEM=
{
	2740,		//u slucaju sudara na prvom delu putanje do mape, ide do zida
	2740, 		//pokusava uz zid da dodje do mape
	2400,		//dolazi do drveta i pokusava tako do mape							//2400
	2400		//pozicija odakle nastavlja sa kupljenjem drugog dela naseg totema	//2400
};	

unsigned int pathYRedAlt[brojPozicijaCrvenoAlt] PROGMEM=
{
	3350, //1650,
	2570, //2480,   //MWFM 2540
	2550, //2450,
	3320 //1680
};

signed char directionRedAlt[brojPozicijaCrvenoAlt] PROGMEM=
{
	 1,
	-1,
	 1,
	 1	
};

void blueSide(void)
{
	/************************************************************************/
	/*			PROMENLJIVE ZA KONTROLU MECA                                */
	/************************************************************************/
	eNesto state = FAST_AND_FURIOUS;
	eBool totemCollisionFlag = FALSE;
	
	unsigned char position = 0, newPosition = 0, collisionCounter = 0, tacticCombination;
	int speed = NORMALSPEED;
	char detectionFlags = ALL_DETECTIONS_ON;
	
	if(prekTaktika1Provera() && prekTaktika2Provera())
		tacticCombination = 1;//T1T2MK
	else if((!prekTaktika1Provera()) && (!prekTaktika2Provera()))
		tacticCombination = 0;	//T1T2MT3K
	else if(prekTaktika1Provera())
		tacticCombination = 2;	//T1T2T3MK
	else
		tacticCombination = 3;	//T1T2KM
	

	/************************************************************************/
	/*		POCETNA POZICIJA ZA LJUBICASTU STRANU                           */
	/************************************************************************/
	setStartPosition(2789,1312,349);  
	otvoriSkupljac();

	while(1)
	{
		switch(state)
		{
			/************************************************************************/
			/*	GLAVNE PUTANJE ZA MEC                                               */
			/************************************************************************/	
			case FAST_AND_FURIOUS:
				for(position = newPosition; position < brojPozicijaPlavo; position++)
				{
					if(gotoXY(pgm_read_word(&pathXBlue[position]),pgm_read_word(&pathYBlue[position]),pgm_read_byte(&directionBlue[position]),speed,NORMALPRECISION,detectionFlags))
					{
						newPosition = position;
						
						state = COLLISION;
						speed = NORMALSPEED;
						detectionFlags = ALL_DETECTIONS_OFF;
						
						break;	
					}//end collision if
					
					speed = NORMALSPEED;
					detectionFlags = ALL_DETECTIONS_ON;
					/************************************************************************/
					/*   ISPRED PRVOG DELA NASEG TOTEMA, PRE OKRETANJA KA NJEMU             */
					/************************************************************************/
					if(position == 0)
					{	
						if(tasRobotNapredProvera())		//u slucaju da je neki pametnjakovic odlucio da odmah dodje do nas, taster ce biti pritisnut
						{
							pauza_ms(20);
							moveOnDirection(150,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//guramo ga da nam ne smeta
							pauza_ms(20);
							moveOnDirection(150,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//vracamo se nazad		
						}//end taster if
						
						otvoriSkupljac();
						
						setUgao(245);	//okrece se ka totemu				
						pauza_ms(10);	
						
						moveOnDirection(150,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//prilazi totemu ali ne skroz, pre cimanja diska
						skupljacCim(MOTOR_SKUPLJAC_LEVI,30);	//cima levi skupljac i tako se stiti od diska
						
						moveOnDirection(140,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//160		//dolazi skroz do drvenog braon govna
						
						collisionCounter = 0;
						while(setUgao(191))	//cisti sa totema, AKO NE MOZE VRACA SE U PRETHODNI UGAO, UKLJUCUJE CIM I NASTAVLJA
						{
							if(++collisionCounter == 3)
							{
								collisionCounter = 0;
								moveOnDirection(160,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
								setUgao(245);
								moveOnDirection(130,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
								continue;	
							}	
													
							skupljacCim(MOTOR_SKUPLJAC_LEVI,30);
							pauza_ms(50);
						}//end while(setUgao)
						
						collisionCounter = 0;
						
						/************************************************************************/
						/* kupi sta je ocistio, sve dok ne dodje gde treba                      */
						/************************************************************************/
	
						moveOnDirection(270,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME); //ide malo napred i tako kupi sta je srusio
						
						while(setUgao(210))			//malo koriguje ugao, da bi mogao skupiti i ostale
						{
							moveOnDirection(30,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						}	
						
						moveOnDirection(350,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);  //IDE skroz do naseg broda, tu mora da dodje!
						
						setUgao(180);	//okrece se pre ulaska u cilj
						
						pauza_ms(5);
						
						moveOnDirection(260,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//ulazim u cilj
						pauza_ms(5);
						moveOnDirection(210,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//izlazim iz cilja
						
						zatvoriSkupljac();		//zatvara govna
						
						continue;
					}//end position if	
					
					/************************************************************************/
					/*   IDE DA KUPI DRUGI DEO TOTEMA, PRE KOSE KRETNJE                     */
					/************************************************************************/
					if(position == 1)
					{
						collisionCounter = 0;
						
						continue;
					}
					
					/************************************************************************/
					/*	NALAZI SE ISPRED DRUGOG DELA NASEG TOTEMA                           */
					/************************************************************************/
					if(position == 2)
					{				
						setCommandInt('V',NORMALSPEED);
						setUgao(105);	//okrece se ka totemu
						otvoriSkupljac();
						
						moveOnDirection(280,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME); //285 prilazi totemu pre cimanja diska		
						
						skupljacCim(MOTOR_SKUPLJAC_DESNI,45);
						
						if(moveOnDirection(60,1,NORMALSPEED,ALL_DETECTIONS_OFF,ONE_SECOND_BREAK_TIME))	//skroz prilazi totemu  70
						{
							skupljacCim(MOTOR_SKUPLJAC_LEVI,40);
							moveOnDirection(10,1,NORMALSPEED,ALL_DETECTIONS_OFF,ONE_SECOND_BREAK_TIME);
						}						
							
						skupljacCim(MOTOR_SKUPLJAC_DESNI,30);	//CIMA (DVA PUTA) DA SE NE BI ZAPUCO U DISK NA TOTEMU PRILIKOM CISCENJA
						pauza_ms(100);
						skupljacCim(MOTOR_SKUPLJAC_DESNI,30);
						
						while(setUgao(170))	//cisti sa totema  //++++
						{
							skupljacCim(MOTOR_SKUPLJAC_DESNI,30);	//ako ne moze da se okrene ukljucuje cim i pokusava ponovo
							pauza_ms(10);
						}
						
						moveOnDirection(270,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//ide napred prema cilju, mora da dodje tu
						pauza_ms(5);
						
						while(setUgao(130))	//okrece se prema totemu da pokupi ono sto mu je ispalo	130
						{
							moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);		//TESTIRATI!!!
						}
						
						moveOnDirection(510,1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME); //ide jos malo prema cilju
						
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,135);	//delimicno zatvara skupljace da ne bi sebe zeznuo
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,35);
						pauza_ms(110);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);	
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						
						moveOnDirection(135,1,NORMALSPEED,TASTER_SUDAR_ON,TWO_SECOND_BREAK_TIME);	//PRE ULASKA U CILJ
						
						setUgao(180);	//okrece se da udje u cilj
						
						moveOnDirection(140,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//ulazi u cilj
						pauza_ms(5);
						
						moveOnDirection(320,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//izlazim iz cilja
						
						setUgao(270);	//postavlja se da da moze da ide da kupi mapu
						
						moveOnDirection(200,-1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME);	//izlazi malo da se na putu do mape ne bi spuco u totem
						
						zatvoriSkupljac();
						
						collisionCounter = 0;
						
						if((tacticCombination == 2) || ((tacticCombination == 0) && (totemCollisionFlag == TRUE)))	
						{
							setUgao(270);
							otvoriSkupljac();
							newPosition = 4;//ide na protivnicki totem
							state = FAST_AND_FURIOUS;	
							
							break;
						}
						
						if((tacticCombination == 3) || ((tacticCombination == 1) && (totemCollisionFlag == TRUE)))	//ide da krade
						{
							setUgao(270);
							
							while(gotoXY(2700,1835,-1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))	//ide na poziciju odakle krece da krade
							{
								pauza_ms(50);
								moveOnDirection(20,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
								pauza_ms(500);
							}
								
							newPosition = 7;
							state = FAST_AND_FURIOUS;
							detectionFlags = ALL_DETECTIONS_ON;
							break;	
						}
						
						continue;
					}//end position if
					
					/************************************************************************/
					/*  DOSAO JE KOD MAPE                                                   */
					/************************************************************************/
					if(position == 3)
					{
						zatvoriSkupljac();	
						
						//realizacija zastite za taktiku kada ide da krade ili na protivnicki totem pre mape
						//ako je nesto detektovao da se ne okrece
						//ako regularno ide po mapu ovaj uslov ce biti ispunjen,
						//to je posle drugog dela totema ili posle istovaranja u start
						//ako tu ide prinudno, tj. zbog sudara onda nece biti ispunjen
						//if((readUgaoStepeni()>90) && (readUgaoStepeni()<270))		
						//{
						if((tacticCombination != 3)  && (tacticCombination != 2))	//ulazi ovde samo ako je ovo planski, tj. ne zbog sudara
						{
							setUgao(180);				//okrece se paralelno mapi
						
							moveOnDirection(100,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//guram disk koji se mozda nalazi iza
							pauza_ms(5);
							moveOnDirection(100,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//vracam se nazad
						}											
					
						if(tacticCombination == 3)	//ako je usao u slucaju sudara menjam mu taktiku
							tacticCombination = 1;
						if(tacticCombination == 2)
							tacticCombination = 0;
							
						setUgao(272);				//okrece se da moze da pokupi mapu
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,SKUPLJAC_MAPE_BRZINA);
						moveOnDirection(1000,-1,LOWSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME); //prilazi mapi
						
						pokupiMapu();	
						
						moveOnDirection(320,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);	//ponovo aktivira aktuator da se ne bi mapa vukla po podu
						pauza_ms(600);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);
						
						if(totemCollisionFlag == TRUE)	//indikacija da nije pokupio drugi deo naseg totema
						{
							state = RIDE_THE_LIGHTING;	//ide da kupi drugi deo naseg totema
							newPosition = 3;
							
							break;	
						}
						
						if(tacticCombination != 0)	
						{
							newPosition = 7;	//ide da krade
							state = FAST_AND_FURIOUS;
							detectionFlags = ALL_DETECTIONS_ON;;
							break;
						}
						
						otvoriSkupljac();	
						//nastavlja dalje, na protivnicki totem
						continue;
					}//end position if
					
					/************************************************************************/
					/*  NALAZI SE ISPRED PROTIVNICKOG TOTEMA                                */
					/************************************************************************/
					if(position == 4)
					{
						setUgao(245);   //nalazi se ispred protivnickog totema, okrece se da bi mu mogao prici
						
						moveOnDirection(140,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						
						skupljacCim(MOTOR_SKUPLJAC_LEVI,35);	//brani se od onog sugavog diska na uglu
						
						moveOnDirection(95,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//prilazi skroz
						setUgao(195);  //cisti sa totema  190
						moveOnDirection(150,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//kupi
						
						setUgao(165); //155	//ispravlja se
						moveOnDirection(300,1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME); //ide malo napred ka startu
						
						continue;
					}//end position if
					
					/************************************************************************/
					/*   SKORO PA ISPRED NASEG STARTA- KABINE                               */
					/************************************************************************/
					if(position == 5)
					{
						if(tasSkupljacDesniOtvoren() || tasSkupljacLeviOtvoren())	//u slucaju branja protivnickog totema 
						{
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,35);	//delimicno zatvara govna pre ulaska u start - kabinu
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,135);
							pauza_ms(110);
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
						}else         //u slucaju kradje sa protivnickog cilja
						{
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,135);	//delimicno otvara govna pre ulaska u start - kabinu
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,35);
							pauza_ms(170);
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);	
						}		
						
						continue;	
					}//end position if
					
					/************************************************************************/
					/*	U STARTU                                                            */
					/************************************************************************/
					if(position == 6)
					{
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,50);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,150);
						pauza_ms(50);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);	
						
						if((tacticCombination == 2) || (tacticCombination == 3))	//ide na mapu
						{
							moveOnDirection(220,-1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME);
							zatvoriSkupljac();
							
							newPosition = 1;
							state = RIDE_THE_LIGHTING;
							collisionCounter = 0;
							
							break;	
						}
						
						continue;				
					}//end position if
					
					/************************************************************************/
					/*	IDE NA DRUGI KRAJ STOLA, PRE KRADJE                                 */
					/************************************************************************/
					if(position == 7)
					{
						zatvoriSkupljac();
						detectionFlags = ALL_DETECTIONS_ON;
						continue;
					}
					
					/************************************************************************/
					/*		ISPRED PROTIVNICKOG BRODA                                       */
					/************************************************************************/
					if(position == 8)
					{
						setUgaoNula();	//okrecem se ka protivnickom brodu
						otvoriSkupljac();	//otvaram govna
						
						moveOnDirection(160,1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,150);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,50);
						pauza_ms(70);
						otvoriSkupljac();
						moveOnDirection(100,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,150);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,50);
						pauza_ms(80);
						otvoriSkupljac();
						moveOnDirection(90,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,150);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,50);
						pauza_ms(70);
						otvoriSkupljac();
						moveOnDirection(40,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);

					
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,140);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,40);
						pauza_ms(350);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
						pauza_ms(10);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,10);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,110);
						moveOnDirection(350,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//izlazim iz broda  290
						
						while(gotoXY(2620,readY()-60,1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))	//idem na poziciju odakle krecem ka svojoj kabini- startu
						{
							pauza_ms(5);
							moveOnDirection(10,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
							pauza_ms(1000);
						}
						
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
						
						state = FAST_AND_FURIOUS;
						newPosition = 5;
						break;	
					}//end position if	
				}//end position for()
			
				break;
			
			/************************************************************************/
			/*		ALTERNATIVNE I POMOCNE KRETNJE                                  */
			/************************************************************************/	
			case RIDE_THE_LIGHTING:
				for(position = newPosition; position<brojPozicijaPlavoAlt;position++)
				{
					if(gotoXY(pgm_read_word(&pathXBlueAlt[position]),pgm_read_word(&pathYBlueAlt[position]),pgm_read_byte(&directionBlueAlt[position]),NORMALSPEED,NORMALPRECISION,detectionFlags))
					{
						state=COLLISION;
						newPosition = position + 100;
						speed = NORMALSPEED;
						detectionFlags = ALL_DETECTIONS_ON;
		
						break;	
					}//end collison if	
					
					detectionFlags = ALL_DETECTIONS_ON;
					speed = NORMALSPEED;
					
					/************************************************************************/
					/* POZICIJA BLIZU ZIDA ODAKLE KRECE NA MAPU                            */
					/************************************************************************/
					if(position == 0)
					{
						continue;		
					}//end position if
					
					/************************************************************************/
					/* NALAZI SE ISPRED MAPE PARALELNO SA ZIDOM                             */
					/************************************************************************/
					if(position == 1)
					{
						moveOnDirection(100,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						pauza_ms(50);
						moveOnDirection(100,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						
						setUgao(270);
						
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,SKUPLJAC_MAPE_BRZINA);
						moveOnDirection(1000,-1,LOWSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						
						pokupiMapu();
						
						moveOnDirection(320,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);	//ponovo aktivira aktuator da se ne bi mapa vukla po podu
						pauza_ms(1000);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);
						
						if(totemCollisionFlag == TRUE)	//ako je drugi deo naseg totema nepokupljen
						{
							state = RIDE_THE_LIGHTING;	//IDEM NA POZICIJU ODAKLE SE VRACAM NA DRUGI DEO TOTEMA, NEMA VEZE STO JE DIREKTNO ISPOD OVAKO ZNAM STA RADIM :-)
							newPosition = 3;
							detectionFlags = ALL_DETECTIONS_ON;
								
							break;
						}//end totem if	
						
						if(tacticCombination != 0)	
						{
							state = FAST_AND_FURIOUS;
							newPosition = 7;	//prelazim na kradju
							tacticCombination = 0;  //PROVERITI
							detectionFlags = ALL_DETECTIONS_ON;
							break;	
						}//end tactisCombination if
						
						state = FAST_AND_FURIOUS;
						newPosition = 4;	//pozicija za kupljenje protivnickog totema
						
						break;
					}//end position if
					
					/************************************************************************/
					/*	NALAZI SE KOD DRVETA                                                */
					/************************************************************************/
					if(position == 2)
					{
						setUgao(270);	//okrecem se ka mapi
						
						if(gotoXY(readX()+350,readY(),-1,NORMALSPEED,NORMALPRECISION,TASTER_SUDAR_ON))	//pokusava da dodje blizu mape	
						{
							pauza_ms(50);
							moveOnDirection(70,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	
							pauza_ms(300);
							
							if(totemCollisionFlag == TRUE)	//ako je drugi deo naseg totema nepokupljen
							{
								state = RIDE_THE_LIGHTING;	//IDEM NA POZICIJU ODAKLE SE VRACAM NA DRUGI DEO TOTEMA, NEMA VEZE STO JE DIRETNO ISPOD OVAKO ZNAM STA RADIM :-)
								newPosition = 3;
								detectionFlags = ALL_DETECTIONS_ON;
								totemCollisionFlag = FALSE;
								
								break;
							}//end totem if		
						}//end gotoxy If
						
						pauza_ms(50);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,SKUPLJAC_MAPE_BRZINA);
						moveOnDirection(1000,-1,LOWSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						
						pokupiMapu();
						
						moveOnDirection(320,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);	//ponovo aktivira aktuator da se ne bi mapa vukla po podu
						pauza_ms(1000);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);
						
						if(totemCollisionFlag == TRUE)	//ako je drugi deo naseg totema nepokupljen
						{
							state = RIDE_THE_LIGHTING;	//IDEM NA POZICIJU ODAKLE SE VRACAM NA DRUGI DEO TOTEMA, NEMA VEZE STO JE DIRETNO ISPOD OVAKO ZNAM STA RADIM :-)
							newPosition = 3;
							detectionFlags = ALL_DETECTIONS_ON;
								
							break;
						}//end totem if	
						
						if(tacticCombination != 0)	//prelazim na kradju
						{
							state = FAST_AND_FURIOUS;
							newPosition = 7;
							detectionFlags = ALL_DETECTIONS_ON;
							break;	
						}//end tactisCombination if
						
						state = FAST_AND_FURIOUS;	//PROTIVNICKI TOTEM
						newPosition = 4;
						
						break;
					}//end position if
					
					/************************************************************************/
					/* NASTAVLJA SA KUPLJENJEM DRUGOG DELA NASEG TOTEMA                     */
					/************************************************************************/
					if(position == 3)
					{
						setUgao(90);
						collisionCounter = 0;
						
						state = FAST_AND_FURIOUS;
						newPosition = 1;
						
						speed = NORMALSPEED;
						detectionFlags = ALL_DETECTIONS_ON;
						
						break;
					}//end position if					
				}//end position for
				break;
				
/************************************************************************************************************************************************************///////				
			/************************************************************************/
			/*	KRETNJE U SLUCAJU SUDARA                                              */
			/************************************************************************/	
			case COLLISION:
			
				/************************************************************************/
				/*		DETEKCIJA ODMAH PRI IZLASKU IZ STARTA                           */
				/************************************************************************/
				if(newPosition == 0)
				{
					if(tasRobotNapredProvera())	//ovde ce uci ako je neko bas brzo doleteo do nas, a posto nam smeta dobice svoje
					{
						pauza_ms(10);
						moveOnDirection(150,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						pauza_ms(500);
						
						speed = UPSPEED;	//za guranje, steta sto nemamo 30kg kao neki seljaci
						detectionFlags = ALL_DETECTIONS_OFF;
						state = FAST_AND_FURIOUS;	
						
						break;
					}//end taster if
					
					zatvoriSkupljac();	
					pauza_ms(300);
					
					speed = NORMALSPEED + 100;
					detectionFlags = TASTER_SUDAR_ON;
					
					state = FAST_AND_FURIOUS;	//kupljenje prvog dela totema
					newPosition = 0;
					
					break;	
				}//end position if
				
				/******************************************************************************/
				/*    SUDAR PRI DOLASKU NA DRUGI KRAJ STOLA PRE KUPLJENJA DRUGOG DELA TOTEMA  */
				/******************************************************************************/
				if(newPosition == 1)
				{
					if(++collisionCounter < 3)	//pokusavam 3 puta da prodjem
					{
						moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						pauza_ms(500);
						
						detectionFlags = TASTER_SUDAR_ON;
						state = FAST_AND_FURIOUS;	//kupljenje drugog dela totema
						newPosition = 1;
					
						break;
					}//end collisionCounter if
					
					//zatvoriSkupljac();	NEPOTREBNO?	
				
					while(gotoXY(2320,readY(),1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))		//idem na poziciju odakle mogu da krenem na mapu
					{
						pauza_ms(5);
						moveOnDirection(10,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						pauza_ms(700);
					}//end while if
					
					if(totemCollisionFlag == TRUE)	//ako je vec pokupio mapu
					{
						while(gotoXY(2615,readY(),1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))	//ide na poziciju odakle nastavlja dalje
							pauza_ms(500);	
						
						if(tacticCombination == 0)
						{
							state = FAST_AND_FURIOUS; //totem
							newPosition = 4;
							otvoriSkupljac();	
						}else
						{
							state = FAST_AND_FURIOUS;//kradja
							newPosition = 7;
							detectionFlags = ALL_DETECTIONS_ON;	
						}
							
						break;	
					}
					
					detectionFlags = ALL_DETECTIONS_ON;
					totemCollisionFlag = TRUE;	//ako je mapa nepokupljena idem na nju
					state = FAST_AND_FURIOUS;								
					newPosition = 3;
					
					if(tacticCombination == 3)
						tacticCombination = 1;
					if(tacticCombination == 2)
						tacticCombination = 0;
						
					break;
				}//end position if
				
				/************************************************************************/
				/*	SUDAR KADA IDE KOSIM PUTEM DO DRUGOG DELA TOTEMA                    */
				/************************************************************************/
				if(newPosition == 2)
				{
					if(collisionCounter == 0)	//ako prvi put detektujem
					{
						pauza_ms(900);
						state = FAST_AND_FURIOUS;
						newPosition = 2;	//pokusavam ponovo sa malom brzinom i bez sharpa
						speed = LOWSPEED;
						detectionFlags = TASTER_SUDAR_ON;
						collisionCounter++;
						
						break;
					}
					
					if(++collisionCounter>2)	//ako bas ne moze da dodje
					{
						pauza_ms(500);	
						
						gotoXY(1620,1680,1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_OFF);	//vracam se tako da bih mogao da kupim mapu
						detectionFlags = ALL_DETECTIONS_ON;
						collisionCounter = 0;
						pauza_ms(300);
						while(gotoXY(2320,1680,-1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))		//idem na prvi deo stola, tu odlucujem sta dalje   /-1
						{
							if(++collisionCounter>=5)	//ako ni tu ne moze doci vraca se da pokusa ponovo sa totemom
							{
								state = FAST_AND_FURIOUS;
								speed = NORMALSPEED;
								detectionFlags = ALL_DETECTIONS_ON;
								collisionCounter = 0;
								newPosition = 1;
								
								break;	
							}//end if
							
							pauza_ms(10);
							moveOnDirection(20,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
							pauza_ms(500);
						}//end while()
						
						collisionCounter = 0;
						
						
						if(totemCollisionFlag == FALSE)	
						{
							totemCollisionFlag = TRUE;	//ukljucujem indikaciju da nije pokupio drugi deo totema
							state = FAST_AND_FURIOUS;	//i idem na mapu
							newPosition = 3;
							
							if(tacticCombination == 3)
								tacticCombination = 1;
							if(tacticCombination == 2)
								tacticCombination = 0;
							
							break;
						}
						
						if(tacticCombination == 0)
						{
							otvoriSkupljac();
							state = FAST_AND_FURIOUS; //totem
							newPosition = 4;
							
						}else
						{
							gotoXY(2700,1835,-1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON);
							state = FAST_AND_FURIOUS; //kradja
							newPosition = 7;	
							detectionFlags =	TASTER_SUDAR_ON;
						}						
							
						break;
						
					}//end collision if
					
					pauza_ms(10);
					moveOnDirection(30,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
					pauza_ms(500);	
					
					newPosition = 2;	//pokusavam ponovo
					state = FAST_AND_FURIOUS;
					speed = NORMALSPEED;
					detectionFlags = TASTER_SUDAR_ON;
					
					break;		
				}//end position if
				
				/************************************************************************/
				/*		SUDAR NA PUTU DO MAPE                                           */
				/************************************************************************/
				if(newPosition == 3)
				{
					pauza_ms(600);	

					zatvoriSkupljac();
					
					collisionCounter = 1;		//ide do zida
					detectionFlags = TASTER_SUDAR_ON;		//TESTIRATI
					speed = NORMALSPEED;
					
					state = RIDE_THE_LIGHTING;	//dolazi do zida, pokusace paralelno mapi da dodje do nje
					newPosition = 0;
					
					break;
				}//end position if
				
				/************************************************************************/
				/* SUDAR NA PUTU DO PROTIVNICKOG TOTEMA				                    */
				/************************************************************************/
				if(newPosition == 4)
				{
					if((tacticCombination == 2) && (readY()>2530))	//ako je presao mapu odlazi da kupi nju(mapa prethodno nije pokupljena)
					{						
						moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						state = FAST_AND_FURIOUS;
						newPosition = 3;
						
						detectionFlags = ALL_DETECTIONS_ON;
						
						pauza_ms(500);
						
						break;
					}
					
					pauza_ms(500);	//u suprotnom pokusava ponovo
					detectionFlags = ALL_DETECTIONS_ON;
					state = FAST_AND_FURIOUS;
					
					break;
				}//end position if
				
				/************************************************************************/
				/* SUDAR NA PUTU DO PROTIVNICKOG BRODA				                    */
				/************************************************************************/
				if(newPosition == 7)
				{
					if(((tacticCombination) == 3) && (readY()>2530))	//ako je presao mapu odlazi da kupi nju(mapa prethodno nije pokupljena)
					{
						moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						state = FAST_AND_FURIOUS;
						newPosition = 3;
						PORTG = 0xFF;
						detectionFlags = ALL_DETECTIONS_ON;
						pauza_ms(500);
						
						break;
					}
					
					pauza_ms(500);
					
					detectionFlags = ALL_DETECTIONS_ON;
					state = FAST_AND_FURIOUS;
					
					break;
				}//end position if
				
				if(newPosition == 8)
				{
					pauza_ms(500);
					
					
					if(tacticCombination == 3)	//ako je presao mapu odlazi da kupi nju(mapa prethodno nije pokupljena)
					{
						gotoXY(2700,readY(),1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_OFF);
						state = FAST_AND_FURIOUS;
						newPosition = 3;
						
						detectionFlags = ALL_DETECTIONS_ON;
						
						break;
					}
				}
				
				/************************************************************************/
				/*	SUDAR PRILIKOM DOLASKA DO MAPE PARALELNO ZIDU                      */
				/************************************************************************/
				if(newPosition == 101)
				{
					pauza_ms(500);
					if((++collisionCounter > 3) && (totemCollisionFlag == TRUE))	//ako pokusava 4. put da dodje a nije pokupio drugi deo totema
					{
						state = RIDE_THE_LIGHTING;
						newPosition = 3;
						totemCollisionFlag = FALSE;
						break;	
					}//end collisionCounter if	
					
					pauza_ms(500);
					
					gotoXY(2740,1760,1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_OFF);	//IDE DO NEKE TACKE
					
					state = RIDE_THE_LIGHTING;	//vraca se do zida, i pokusava da ide do drveta
					newPosition = 2;
					detectionFlags = TASTER_SUDAR_ON;
					
					break;	
				}//end position if
				
				/************************************************************************/
				/*	SUDAR PRILIKOM DOLASKA DO DRVETA                                    */
				/************************************************************************/
				if(newPosition == 102)
				{
					pauza_ms(500);
					if((++collisionCounter > 3) && (totemCollisionFlag == TRUE))	//ako pokusava 4. put da dodje a nije pokupio drugi deo totema
					{
						collisionCounter = 0;
						
						totemCollisionFlag = FALSE;
						state = RIDE_THE_LIGHTING;	//ide do pozicije odakle krece na drugi deo totema
						newPosition = 3;

						break;	
					}//end collisionCounter if	
					
					state = RIDE_THE_LIGHTING;	//ide do zida, ponovo pokusava paralelno mapi da dodje do nje
					newPosition = 0;
					
					break;
				}//end position if
				
				if(newPosition>=100)
				{
					detectionFlags = ALL_DETECTIONS_ON;
					pauza_ms(600);
					newPosition -= 100;
					state = RIDE_THE_LIGHTING;
					
					break;	
				}//end position if
				
				pauza_ms(600);
				detectionFlags = ALL_DETECTIONS_ON;
				state = FAST_AND_FURIOUS;
				
				break;				
		}//end switch(state)
	}//end while(1)
}//end blueSide

/***********************************************************************************************************************************************************/
/*												CRVENA STRANA																							   */
/*												CRVENA STRANA																							   */
/*												CRVENA STRANA                                                                                              */
/***********************************************************************************************************************************************************/

void redSide(void)
{
	/************************************************************************/
	/*			PROMENLJIVE ZA KONTROLU MECA                                */
	/************************************************************************/
	eNesto state = FAST_AND_FURIOUS;
	eBool totemCollisionFlag = FALSE;
	
	unsigned char position = 0, newPosition = 0, collisionCounter = 0, tacticCombination;
	int speed = NORMALSPEED;
	char detectionFlags = ALL_DETECTIONS_ON;
	
	if(prekTaktika1Provera() && prekTaktika2Provera())
		tacticCombination = 1;//T1T2MK
	else if((!prekTaktika1Provera()) && (!prekTaktika2Provera()))
		tacticCombination = 0;	//T1T2MT3K
	else if(prekTaktika1Provera())
		tacticCombination = 2;	//T1T2T3MK
	else
		tacticCombination = 3;	//T1T2KM
	

	/************************************************************************/
	/*		POCETNA POZICIJA ZA CRVENU STRANU		                        */
	/************************************************************************/
	setStartPosition(2789,3688,191);  
	otvoriSkupljac();
	while(1)
	{
		switch(state)
		{
			/************************************************************************/
			/*	GLAVNE PUTANJE ZA MEC                                               */
			/************************************************************************/	
			case FAST_AND_FURIOUS:
				for(position = newPosition; position < brojPozicijaPlavo; position++)
				{
					if(gotoXY(pgm_read_word(&pathXRed[position]),pgm_read_word(&pathYRed[position]),pgm_read_byte(&directionRed[position]),speed,NORMALPRECISION,detectionFlags))
					{
						newPosition = position;
						
						state = COLLISION;
						speed = NORMALSPEED;
						detectionFlags = ALL_DETECTIONS_OFF;
						
						break;	
					}//end collision if
					
					speed = NORMALSPEED;
					detectionFlags = ALL_DETECTIONS_ON;
					/************************************************************************/
					/*   ISPRED PRVOG DELA NASEG TOTEMA, PRE OKRETANJA KA NJEMU             */
					/************************************************************************/
					if(position == 0)
					{
						if(tasRobotNapredProvera())		//u slucaju da je neki pametnjakovic odlucio da odmah dodje do nas, taster ce biti pritisnut
						{
							pauza_ms(20);
							moveOnDirection(150,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//guramo ga da nam ne smeta
							pauza_ms(20);
							moveOnDirection(150,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//vracamo se nazad		
						}//end taster if
						
						otvoriSkupljac();
						setCommandInt('V',NORMALSPEED);
						setUgao(295);	//okrece se ka totemu				
				
						pauza_ms(10);	
						
						moveOnDirection(130,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//prilazi totemu ali ne skroz, pre cimanja diska
						skupljacCim(MOTOR_SKUPLJAC_DESNI,30);	//cima levi skupljac i tako se stiti od diska
						
						moveOnDirection(145,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);			//150 dolazi skroz do drvenog braon govna
						
						collisionCounter = 0;
						while(setUgao(352))	//348 cisti sa totema, AKO NE MOZE VRACA SE U PRETHODNI UGAO, UKLJUCUJE CIM I NASTAVLJA
						{				//193
							if(++collisionCounter == 4)
							{
								collisionCounter = 0;
								moveOnDirection(160,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
								setUgao(295);
								moveOnDirection(140,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
								continue;	
							}	
													
							skupljacCim(MOTOR_SKUPLJAC_LEVI,50);
							pauza_ms(50);
						}//end while(setUgao)
						
						collisionCounter = 0;
						
						/************************************************************************/
						/* kupi sta je ocistio, sve dok ne dodje gde treba                      */
						/************************************************************************/
	
						moveOnDirection(280,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME); //ide malo napred i tako kupi sta je srusio
						
						while(setUgao(329))			//malo koriguje ugao, da bi mogao skupiti i ostale
						{
							moveOnDirection(30,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						}	
						
						moveOnDirection(350,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);  //IDE skroz do naseg broda, tu mora da dodje!
						
						setUgaoNula();	//okrece se pre ulaska u cilj
						
						pauza_ms(5);
						
						moveOnDirection(260,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//ulazim u cilj
						pauza_ms(5);
						moveOnDirection(220,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//izlazim iz cilja
						
						zatvoriSkupljac();		//zatvara govna
						
						continue;
					}//end position if	
					
					/************************************************************************/
					/*   IDE DA KUPI DRUGI DEO TOTEMA, PRE KOSE KRETNJE                     */
					/************************************************************************/
					if(position == 1)
					{
						collisionCounter = 0;
						
						continue;
					}
					
					/************************************************************************/
					/*	NALAZI SE ISPRED DRUGOG DELA NASEG TOTEMA                           */
					/************************************************************************/
					if(position == 2)
					{				
						setCommandInt('V',NORMALSPEED);
						setUgao(75);	//okrece se ka totemu  //MENJANO!  90
						otvoriSkupljac();
						
						//moveOnDirection(160,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME); //prilazi totemu pre cimanja diska		
						moveOnDirection(290,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);  //165  MENJANO!
										
						skupljacCim(MOTOR_SKUPLJAC_LEVI,45);
						
						if(moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,ONE_SECOND_BREAK_TIME))	//skroz prilazi totemu  70  MENJANO!
						{	
							skupljacCim(MOTOR_SKUPLJAC_DESNI,40);
							moveOnDirection(10,1,NORMALSPEED,ALL_DETECTIONS_OFF,ONE_SECOND_BREAK_TIME);
						}						
							
						skupljacCim(MOTOR_SKUPLJAC_LEVI,30);	//CIMA (DVA PUTA) DA SE NE BI ZAPUCO U DISK NA TOTEMU PRILIKOM CISCENJA
						pauza_ms(90);
						skupljacCim(MOTOR_SKUPLJAC_LEVI,30);
						
						while(setUgao(15))	//cisti sa totema
						{			//170
							skupljacCim(MOTOR_SKUPLJAC_DESNI,60);	//ako ne moze da se okrene ukljucuje cim i pokusava ponovo
							pauza_ms(10);
						}
						
						moveOnDirection(260,1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME);	//300 ide napred prema cilju, mora da dodje tu MENJANO!
						pauza_ms(5);
						while(setUgao(50))	//okrece se prema totemu da pokupi ono sto mu je ispalo	40
						{
							moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);		//TESTIRATI!!!
						}
						moveOnDirection(510,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME); //530 ide jos malo prema cilju
						
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,135);	//delimicno zatvara skupljace da ne bi sebe zeznuo
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,35);
						pauza_ms(110);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);	
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						
						moveOnDirection(135,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//175 PRE ULASKA U CILJ
						
						setUgaoNula();	//okrece se da udje u cilj
										//140
						moveOnDirection(90,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//ulazi u cilj 140
						pauza_ms(5);
						
						moveOnDirection(320,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//izlazim iz cilja
						
						setUgao(270);	//postavlja se da da moze da ide da kupi mapu
								//270
						moveOnDirection(200,-1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME);	//izlazi malo da se na putu do mape ne bi spuco u totem
						
						zatvoriSkupljac();
						
						collisionCounter = 0;
						
						if((tacticCombination == 2) || ((tacticCombination == 0) && (totemCollisionFlag == TRUE)))	
						{
							setUgao(270);
							otvoriSkupljac();
							newPosition = 4;//ide na protivnicki totem
							state = FAST_AND_FURIOUS;	
							
							break;
						}
						
						if((tacticCombination == 3) || ((tacticCombination == 1) && (totemCollisionFlag == TRUE)))	//ide da krade
						{
							setUgao(270);
											//1835
							while(gotoXY(2700,3165,-1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))	//ide na poziciju odakle krece da krade
							{
								pauza_ms(50);
								moveOnDirection(20,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
								pauza_ms(500);
							}
								
							newPosition = 7;
							state = FAST_AND_FURIOUS;
							detectionFlags = ALL_DETECTIONS_ON;
							break;	
						}
						
						continue;
					}//end position if
					
					/************************************************************************/
					/*  DOSAO JE KOD MAPE                                                   */
					/************************************************************************/
					if(position == 3)
					{
						zatvoriSkupljac();	
						
						//realizacija zastite za taktiku kada ide da krade ili na protivnicki totem pre mape
						//ako je nesto detektovao da se ne okrece
						//ako regularno ide po mapu ovaj uslov ce biti ispunjen,
						//to je posle drugog dela totema ili posle istovaranja u start
						//ako tu ide prinudno, tj. zbog sudara onda nece biti ispunjen
						//if((readUgaoStepeni()>90) && (readUgaoStepeni()<270))		
						//{
						if((tacticCombination != 3)  && (tacticCombination != 2))	//ulazi ovde samo ako je ovo planski, tj. ne zbog sudara
						{
							setUgaoNula();				//okrece se paralelno mapi
						
							moveOnDirection(100,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//guram disk koji se mozda nalazi iza
							pauza_ms(5);
							moveOnDirection(100,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//vracam se nazad
						}											
					
						if(tacticCombination == 3)	//ako je usao u slucaju sudara menjam mu taktiku
							tacticCombination = 1;
						if(tacticCombination == 2)
							tacticCombination = 0;
							
						setUgao(268);				//okrece se da moze da pokupi mapu
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);
						moveOnDirection(1000,-1,LOWSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME); //prilazi mapi
						
						pokupiMapu();	
						
						moveOnDirection(320,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);	//ponovo aktivira aktuator da se ne bi mapa vukla po podu
						pauza_ms(600);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);
						
						if(totemCollisionFlag == TRUE)	//indikacija da nije pokupio drugi deo naseg totema
						{
							state = RIDE_THE_LIGHTING;	//ide da kupi drugi deo naseg totema
							newPosition = 3;
							
							break;	
						}
						
						if(tacticCombination != 0)	
						{
							newPosition = 7;	//ide da krade
							state = FAST_AND_FURIOUS;
							detectionFlags = ALL_DETECTIONS_ON;
							break;
						}
						
						otvoriSkupljac();	
						//nastavlja dalje, na protivnicki totem
						continue;
					}//end position if
					
					/************************************************************************/
					/*  NALAZI SE ISPRED PROTIVNICKOG TOTEMA                                */
					/************************************************************************/
					if(position == 4)
					{
						setUgao(295);   //nalazi se ispred protivnickog totema, okrece se da bi mu mogao prici
								//245
						moveOnDirection(140,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						
						skupljacCim(MOTOR_SKUPLJAC_DESNI,35);	//brani se od onog sugavog diska na uglu
										//95
						otvoriSkupljac();
						moveOnDirection(95,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);	//prilazi skroz
						setUgao(345);  //cisti sa totema  195
						moveOnDirection(150,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	//kupi
						
						setUgao(15); //165	//ispravlja se
						moveOnDirection(300,1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME); //ide malo napred ka startu
						
						continue;
					}//end position if
					
					/************************************************************************/
					/*   SKORO PA ISPRED NASEG STARTA- KABINE                               */
					/************************************************************************/
					if(position == 5)
					{
						if(tasSkupljacDesniOtvoren() || tasSkupljacLeviOtvoren())	//u slucaju branja protivnickog totema 
						{
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,35);	//delimicno zatvara govna pre ulaska u start - kabinu
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,135);
							pauza_ms(110);
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
						}else         //u slucaju kradje sa protivnickog cilja
						{
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,135);	//delimicno otvara govna pre ulaska u start - kabinu
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,35);
							pauza_ms(170);
							PORTG = 0x01;
							saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
							saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);	
						}		
						
						continue;
					}//end position if
					
					/************************************************************************/
					/*	U STARTU                                                            */
					/************************************************************************/
					if(position == 6)
					{
						
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,50);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,150);
						pauza_ms(50);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
						
						if((tacticCombination == 2) || (tacticCombination == 3))	//ide na mapu
						{
							moveOnDirection(220,-1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME);
							zatvoriSkupljac();
							
							newPosition = 1;
							state = RIDE_THE_LIGHTING;
							collisionCounter = 0;
							
							break;	
						}
						
						continue;						
					}//end position if
					
					/************************************************************************/
					/*	IDE NA DRUGI KRAJ STOLA, PRE KRADJE                                 */
					/************************************************************************/
					if(position == 7)
					{
						zatvoriSkupljac();
						detectionFlags = ALL_DETECTIONS_ON;
						
						continue;
					}
					
					/************************************************************************/
					/*		ISPRED PROTIVNICKOG BRODA                                       */
					/************************************************************************/
					if(position == 8)
					{
						setUgao(180);	//okrecem se ka protivnickom brodu
						otvoriSkupljac();	//otvaram govna
						
						moveOnDirection(160,1,NORMALSPEED,ALL_DETECTIONS_ON,NO_BREAK_TIME);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,150);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,50);
						pauza_ms(70);
						otvoriSkupljac();
						moveOnDirection(100,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,150);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,50);
						pauza_ms(80);
						otvoriSkupljac();
						moveOnDirection(90,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,150);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,50);
						pauza_ms(70);
						otvoriSkupljac();
						moveOnDirection(40,1,NORMALSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);

						saljiKinezima(MOTOR_SKUPLJAC_DESNI,140);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,40);
						pauza_ms(350);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
						
						pauza_ms(10);
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,5);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,105);
						moveOnDirection(350,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);	//izlazim iz broda  290
						
						while(gotoXY(2700,readY()+50,1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))	//idem na poziciju odakle krecem ka svojoj kabini- startu
						{
							pauza_ms(5);
							moveOnDirection(10,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
							pauza_ms(1000);
						}
						saljiKinezima(MOTOR_SKUPLJAC_LEVI,0);
						saljiKinezima(MOTOR_SKUPLJAC_DESNI,0);
						state = FAST_AND_FURIOUS;
						newPosition = 5;
						break;	
					}//end position if
					
				}//end position for()
			
				break;
			
			/************************************************************************/
			/*		ALTERNATIVNE I POMOCNE KRETNJE                                  */
			/************************************************************************/	
			case RIDE_THE_LIGHTING:
				for(position = newPosition; position<brojPozicijaPlavoAlt;position++)
				{
					if(gotoXY(pgm_read_word(&pathXRedAlt[position]),pgm_read_word(&pathYRedAlt[position]),pgm_read_byte(&directionRedAlt[position]),NORMALSPEED,NORMALPRECISION,detectionFlags))
					{
						state=COLLISION;
						newPosition = position + 100;
						speed = NORMALSPEED;
						detectionFlags = ALL_DETECTIONS_ON;
		
						break;	
					}//end collison if	
					
					detectionFlags = ALL_DETECTIONS_ON;
					speed = NORMALSPEED;
					
					/************************************************************************/
					/* POZICIJA BLIZU ZIDA ODAKLE KRECE NA MAPU                            */
					/************************************************************************/
					if(position == 0)
					{
						continue;		
					}//end position if
					
					/************************************************************************/
					/* NALAZI SE ISPRED MAPE PARALELNO SA ZIDOM                             */
					/************************************************************************/
					if(position == 1)
					{
						moveOnDirection(100,-1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						pauza_ms(50);
						moveOnDirection(100,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						
						setUgao(270);
								//270
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);
						moveOnDirection(1000,-1,LOWSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						
						pokupiMapu();
						
						moveOnDirection(320,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);	//ponovo aktivira aktuator da se ne bi mapa vukla po podu
						pauza_ms(1000);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);
						
						if(totemCollisionFlag == TRUE)	//ako je drugi deo naseg totema nepokupljen
						{
							state = RIDE_THE_LIGHTING;	//IDEM NA POZICIJU ODAKLE SE VRACAM NA DRUGI DEO TOTEMA, NEMA VEZE STO JE DIREKTNO ISPOD OVAKO ZNAM STA RADIM :-)
							newPosition = 3;
							detectionFlags = ALL_DETECTIONS_ON;
								
							break;
						}//end totem if	
						
						if(tacticCombination !=0)	//prelazim na kradju
						{
							state = FAST_AND_FURIOUS;
							newPosition = 7;
							detectionFlags = ALL_DETECTIONS_ON;
							tacticCombination = 0;  //PROVERITI
							
							break;	
						}//end tactisCombination if
						
						state = FAST_AND_FURIOUS;
						newPosition = 4;
						
						break;
					}//end position if
					
					/************************************************************************/
					/*	NALAZI SE KOD DRVETA                                                */
					/************************************************************************/
					if(position == 2)
					{
						setUgao(270);	//okrecem se ka mapi
								//270
						if(gotoXY(readX()+350,readY(),-1,NORMALSPEED,NORMALPRECISION,TASTER_SUDAR_ON))	//pokusava da dodje blizu mape	
						{
							pauza_ms(50);
							moveOnDirection(70,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);	
							pauza_ms(300);
							
							if(totemCollisionFlag == TRUE)	//ako je drugi deo naseg totema nepokupljen
							{
								state = RIDE_THE_LIGHTING;	//IDEM NA POZICIJU ODAKLE SE VRACAM NA DRUGI DEO TOTEMA, NEMA VEZE STO JE DIRETNO ISPOD OVAKO ZNAM STA RADIM :-)
								newPosition = 3;
								detectionFlags = ALL_DETECTIONS_ON;
								totemCollisionFlag = FALSE;
								break;
							}//end totem if		
						}//end gotoxy If
						
						pauza_ms(50);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,SKUPLJAC_MAPE_BRZINA);
						moveOnDirection(1000,-1,LOWSPEED,ALL_DETECTIONS_OFF,TWO_SECOND_BREAK_TIME);
						
						pokupiMapu();
						
						moveOnDirection(320,1,NORMALSPEED,TASTER_SUDAR_ON,NO_BREAK_TIME);
						
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,150);	//ponovo aktivira aktuator da se ne bi mapa vukla po podu
						pauza_ms(1000);
						saljiKinezima(MOTOR_SKUPLJAC_MAPE,0);
						
						if(totemCollisionFlag == TRUE)	//ako je drugi deo naseg totema nepokupljen
						{
							state = RIDE_THE_LIGHTING;	//IDEM NA POZICIJU ODAKLE SE VRACAM NA DRUGI DEO TOTEMA, NEMA VEZE STO JE DIRETNO ISPOD OVAKO ZNAM STA RADIM :-)
							newPosition = 3;
							detectionFlags = ALL_DETECTIONS_ON;
								
							break;
						}//end totem if	
						
						if(tacticCombination !=0)	//prelazim na kradju
						{
							state = FAST_AND_FURIOUS;
							newPosition = 7;
							detectionFlags = ALL_DETECTIONS_ON;
							break;	
						}//end tactisCombination if
						
						state = FAST_AND_FURIOUS;
						newPosition = 4;
						
						break;
					}//end position if
					
					/************************************************************************/
					/* NASTAVLJA SA KUPLJENJEM DRUGOG DELA NASEG TOTEMA                     */
					/************************************************************************/
					if(position == 3)
					{
						setUgao(90);
								//90
						collisionCounter = 0;
						
						state = FAST_AND_FURIOUS;
						newPosition = 1;
						
						speed = NORMALSPEED;
						detectionFlags = ALL_DETECTIONS_ON;
						
						break;
					}//end position if					
				}//end position for
				break;
				
/************************************************************************************************************************************************************///////				
			/************************************************************************/
			/*	KRETNJE U SLUCAJU SUDARA                                              */
			/************************************************************************/	
			case COLLISION:
			
				/************************************************************************/
				/*		DETEKCIJA ODMAH PRI IZLASKU IZ STARTA                           */
				/************************************************************************/
				if(newPosition == 0)
				{
					if(tasRobotNapredProvera())	//ovde ce uci ako je neko bas brzo doleteo do nas, a posto nam smeta dobice svoje
					{
						pauza_ms(10);
						moveOnDirection(150,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						pauza_ms(500);
						
						speed = UPSPEED;	//za guranje, steta sto nemamo 30kg kao neki seljaci
						detectionFlags = ALL_DETECTIONS_OFF;
						state = FAST_AND_FURIOUS;	
						
						break;
					}//end taster if
					
					zatvoriSkupljac();	
					pauza_ms(300);
					
					speed = NORMALSPEED+100;
					detectionFlags = TASTER_SUDAR_ON;
					
					state = FAST_AND_FURIOUS;	//kupljenje prvog dela totema
					newPosition = 0;
					
					break;	
				}//end position if
				
				/************************************************************************/
				/*   SUDAR PRI DOLASKU NA DRUGI KRAJ STOLA PRE KUPLJENJA DRUGOG DELA TOTEMA  */
				/************************************************************************/
				if(newPosition == 1)
				{
					if(++collisionCounter < 3)	//pokusavam 3 puta da prodjem
					{
						moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						pauza_ms(500);
						
						detectionFlags = TASTER_SUDAR_ON;
						state = FAST_AND_FURIOUS;	//kupljenje drugog dela totema
						newPosition = 1;
					
						break;
					}//end collisionCounter if
					
					//zatvoriSkupljac();	NEPOTREBNO?	
				
					while(gotoXY(2320,readY(),1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))		//idem na poziciju odakle mogu da krenem na mapu
					{
						pauza_ms(5);
						moveOnDirection(10,-1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						pauza_ms(700);
					}//end while if
					
					if(totemCollisionFlag == TRUE)	//ako je vec pokupio mapu
					{
						while(gotoXY(2615,readY(),1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))	//ide na poziciju odakle nastavlja dalje
							pauza_ms(500);	
						
						if(tacticCombination == 0)
						{
							state = FAST_AND_FURIOUS; //totem
							newPosition = 4;
							otvoriSkupljac();	
						}else
						{
							state = FAST_AND_FURIOUS;//kradja
							newPosition = 7;
							detectionFlags = ALL_DETECTIONS_ON;
						}
							
						break;	
					}
					
					detectionFlags = ALL_DETECTIONS_ON;
					totemCollisionFlag = TRUE;	//ako je mapa nepokupljena idem na nju
					state = FAST_AND_FURIOUS;								
					newPosition = 3;
					
					if(tacticCombination == 3)
						tacticCombination = 1;
					if(tacticCombination == 2)
						tacticCombination = 0;
						
					break;
				}//end position if
				
				/************************************************************************/
				/*	SUDAR KADA IDE KOSIM PUTEM DO DRUGOG DELA TOTEMA                    */
				/************************************************************************/
				if(newPosition == 2)
				{
					if(collisionCounter == 0)	//ako prvi put detektujem
					{
						pauza_ms(900);
						state = FAST_AND_FURIOUS;
						newPosition = 2;	//pokusavam ponovo sa malom brzinom i bez sharpa
						speed = LOWSPEED;
						detectionFlags = TASTER_SUDAR_ON;
						collisionCounter++;
						
						break;
					}
					
					if(++collisionCounter>2)	//ako bas ne moze da dodje
					{
						pauza_ms(500);	
						
						gotoXY(1620,3320,1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_OFF);	//vracam se tako da bih mogao da kupim mapu
						detectionFlags = ALL_DETECTIONS_ON;
						collisionCounter = 0;
						pauza_ms(300);
						while(gotoXY(2320,3320,-1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON))		//idem na prvi deo stola, tu odlucujem sta dalje
						{				//1680
							if(++collisionCounter>=5)	//ako ni tu ne moze doci vraca se da pokusa ponovo sa totemom
							{
								state = FAST_AND_FURIOUS;
								speed = NORMALSPEED;
								detectionFlags = ALL_DETECTIONS_ON;
								collisionCounter = 0;
								newPosition = 1;
								
								break;	
							}//end if
							
							pauza_ms(10);
							moveOnDirection(20,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
							pauza_ms(500);
						}//end while()
						
						collisionCounter = 0;
						
						if(totemCollisionFlag == FALSE)	//
						{
							totemCollisionFlag = TRUE;	//ukljucujem indikaciju da nije pokupio drugi deo totema
							state = FAST_AND_FURIOUS;	//i idem na mapu
							newPosition = 3;
							
							if(tacticCombination == 3)
								tacticCombination = 1;
							if(tacticCombination == 2)
								tacticCombination = 0;
							
							break;
						}
						
						if(tacticCombination == 0)
						{
							otvoriSkupljac();
							state = FAST_AND_FURIOUS; //totem
							newPosition = 4;
							
						}else
						{				//1835
							gotoXY(2700,3165,-1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_ON);
							state = FAST_AND_FURIOUS; //kradja
							newPosition = 7;
							detectionFlags = ALL_DETECTIONS_ON;	
						}						
							
						break;
						
					}//end collision if
					
					pauza_ms(10);
					moveOnDirection(30,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
					pauza_ms(500);	
					
					newPosition = 2;	//pokusavam ponovo
					state = FAST_AND_FURIOUS;
					speed = NORMALSPEED;
					detectionFlags = TASTER_SUDAR_ON;
					
					break;		
				}//end position if
				
				/************************************************************************/
				/*		SUDAR NA PUTU DO MAPE                                           */
				/************************************************************************/
				if(newPosition == 3)
				{
					pauza_ms(600);	

					zatvoriSkupljac();
					
					collisionCounter = 0;		//ide do zida
					detectionFlags = TASTER_SUDAR_ON;		//TESTIRATI
					speed = NORMALSPEED;
					
					state = RIDE_THE_LIGHTING;	//dolazi do zida, pokusace paralelno mapi da dodje do nje
					newPosition = 0;
					
					break;
				}//end position if
				/************************************************************************/
				/* SUDAR NA PUTU DO PROTIVNICKOG TOTEMA				                    */
				/************************************************************************/
				if(newPosition == 4)
				{
					if(tacticCombination == 2 && readY()<2470)
					{							//readY()>2530
						moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						state = FAST_AND_FURIOUS;
						newPosition = 3;
						
						detectionFlags = ALL_DETECTIONS_ON;
						
						pauza_ms(500);
						
						break;
					}
					
					pauza_ms(500);
					detectionFlags = ALL_DETECTIONS_ON;
					state = FAST_AND_FURIOUS;
					
					break;
				}//end if
				
				
				/************************************************************************/
				/* SUDAR NA PUTU DO PROTIVNICKOG BRODA				                    */
				/************************************************************************/
				if(newPosition == 7)
				{
					if(tacticCombination == 3 && readY()<2470)
					{							//readY()>2530
						moveOnDirection(50,1,NORMALSPEED,ALL_DETECTIONS_OFF,NO_BREAK_TIME);
						state = FAST_AND_FURIOUS;
						newPosition = 3;
						detectionFlags = ALL_DETECTIONS_ON;
						pauza_ms(500);
						
						break;
					}
					
					pauza_ms(500);
					detectionFlags = ALL_DETECTIONS_ON;
					state = FAST_AND_FURIOUS;
					
					break;
				}//end if
				
				if(newPosition == 8)
				{
					pauza_ms(500);

					if(tacticCombination == 3)	//ako je presao mapu odlazi da kupi nju(mapa prethodno nije pokupljena)
					{
						gotoXY(2700,readY(),1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_OFF);
						state = FAST_AND_FURIOUS;
						newPosition = 3;
						
						detectionFlags = ALL_DETECTIONS_ON;
						
						break;
					}
				}
				
				/************************************************************************/
				/*	SUDAR PRILIKOM DOLASKA DO MAPE PARALELNO ZIDU                      */
				/************************************************************************/
				if(newPosition == 101)
				{
					pauza_ms(500);
					if((++collisionCounter > 3) && (totemCollisionFlag == TRUE))	//ako pokusava 4. put da dodje a nije pokupio drugi deo totema
					{
						state = RIDE_THE_LIGHTING;
						newPosition = 3;
						totemCollisionFlag = FALSE;
						break;	
					}//end collisionCounter if	
					
					pauza_ms(500);
					
					gotoXY(2740,3240,1,NORMALSPEED,NORMALPRECISION,ALL_DETECTIONS_OFF);	//IDE DO NEKE TACKE
								//1760
					state = RIDE_THE_LIGHTING;	//vraca se do zida, i pokusava da ide do drveta
					newPosition = 2;
					detectionFlags = TASTER_SUDAR_ON;
					
					break;	
				}//end position if
				
				/************************************************************************/
				/*	SUDAR PRILIKOM DOLASKA DO DRVETA                                    */
				/************************************************************************/
				if(newPosition == 102)
				{
					pauza_ms(500);
					if((++collisionCounter > 3) && (totemCollisionFlag == TRUE))	//ako pokusava 4. put da dodje a nije pokupio drugi deo totema
					{
						collisionCounter = 0;
						
						totemCollisionFlag = FALSE;
						state = RIDE_THE_LIGHTING;	//ide do pozicije odakle krece na drugi deo totema
						newPosition = 3;
					
						break;	
					}//end collisionCounter if	
					
					state = RIDE_THE_LIGHTING;	//ide do zida, ponovo pokusava paralelno mapi da dodje do nje
					newPosition = 0;
					
					break;
				}
				
				if(newPosition>=100)
				{
					detectionFlags = ALL_DETECTIONS_ON;
					pauza_ms(600);
					newPosition -= 100;
					state = RIDE_THE_LIGHTING;
					
					break;	
				}
				
				pauza_ms(600);
				detectionFlags = ALL_DETECTIONS_ON;
				state = FAST_AND_FURIOUS;
				
				break;				
		}//end switch(state)
	}//end while(1)
	
}//end redSide