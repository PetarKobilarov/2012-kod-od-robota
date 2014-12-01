#ifndef F_CPU
#define F_CPU 11059200UL
#endif

#include <math.h>
#include <util/delay.h>
#include "usart.h"
#include "odometry.h"
#include "funkcije.h"
#include "system.h"
#include "lcd.h"

static char firstTime = 1;
static char ev_angleNotReached = 0;

void Stop(void)
{   
	UART0_Write('S');
    UART0_Read(); 
}

void setCommandInt(unsigned char komanda, int vrednost)
{
    UART0_Write(komanda);
    UART0_Write((unsigned char)(vrednost >> 8));
    UART0_Write((unsigned char) vrednost);
    UART0_Read();  
}


void setCommandShort(unsigned char komanda, short vrednost)
{
    UART0_Write(komanda);
    UART0_Write((unsigned char) vrednost);
    UART0_Read();  
}

//citanje X koordinate
int readX(void)
{
    unsigned char pom;
    signed int msb;
    signed int lsb;
    signed int xp;  //pomocni x 
    signed int x;   

    UART0_Write('X');
    pom = UART0_Read();
    
    if(pom == 'X')
    {
        msb = UART0_Read();
        lsb = UART0_Read();
        
        xp = (signed int)msb;
        xp = xp << 8;
        x = xp | (signed int)lsb;
    } 
    
    return x;
}

//citanje Y koordinate
int readY(void)
{
    unsigned char pom;
    signed int msb;
    signed int lsb;
    signed int yp;  //pomocni y 
    signed int y;   

    UART0_Write('Y');
    pom = UART0_Read();
    
    if(pom == 'Y')
    {
        msb = UART0_Read();
        lsb = UART0_Read();
        
        yp = (signed int)msb;
        yp = yp << 8;
        y = yp | (signed int)lsb;
    } 
    
    return y;
}

//citanje ugla u inkrementima
long readUgaoInkrementi(void)
{
    unsigned char pom;
    signed int msb;
    signed int srednji_msb;
    signed int srednji_lsb;
    signed int lsb;
    long teta_pom;
    long teta;
    
    UART0_Write('O');
    //_delay_ms(1);
    pom = UART0_Read();
    if(pom == 'O')
    {
        msb = UART0_Read();
        srednji_msb = UART0_Read();
        srednji_lsb = UART0_Read();
        lsb = UART0_Read();
        
        teta_pom = (signed int)msb;
        teta_pom = teta_pom << 8;
        teta_pom |= (signed int)srednji_msb;
        teta_pom = teta_pom << 8; 
        teta_pom |= (signed int)srednji_lsb;
        teta_pom = teta_pom << 8;
         
        teta = teta_pom | (signed int)lsb;
    }

    return teta;
}

//citanje ugla u stepenima
int readUgaoStepeni(void)
{
    int teta;
    
    teta = (int)((readUgaoInkrementi() * 360) / PUNKRUG);
    
    return teta;
}

//citanje preostale distance
int readDistanca(void)
{
    unsigned char pom;
    signed int msb;
    signed int lsb;
    signed int dist_pom;
    signed int dist;
    
    UART0_Write('G');
    pom = UART0_Read();
    
    if(pom == 'G')
    {
        msb = UART0_Read();
        lsb = UART0_Read();
        dist_pom = (signed int)msb;
        dist_pom = dist_pom << 8;
        dist = dist_pom | (signed int)lsb;       
        
    }
    
    return dist;
}

//postavljanje pocetnih vrednosti koordinata i ugla
//na pocetku se ovo stavi i onda robot zna gde je i sta je :)
void setStartPosition(int x, int y, int ugao)
{
    long temp_ugao = (long)(((long)ugao * PUNKRUG) / 360L);
    long i;
    
    do
    {
        setCommandInt('I', x);    //sa I se zadaje vrednost X koordinate   
        i = readX();
           
    }while((i < (x-2))  ||  (i > (x+2)));  
    
    do
    {
        setCommandInt('J', y);   // sa J se zadaje vrednost Y koordinate
        i = readY();
        
    }while((i < (y-1))  ||  (i > (y+1)));
    
    do
    {
       setCommandInt('K', ugao);
       i = readUgaoInkrementi();
       
    }while((i < (temp_ugao-1))  ||  (i > (temp_ugao + 1)));
       
}

//racunanje apsolutne vrednosti
int absolut(int number)
{
    if(number < 0) 
        number *= -1;
    else
        number = number;
        
    return number;
}

//funkcija za racunanje ugla ka novoj poziciji
int racunanjeUgla(int x, int y)
{
    int ugao_temp = 0;
    int x_trenutno;
    int y_trenutno;
    
    x_trenutno = readX();
    y_trenutno = readY();
	
	if((x-x_trenutno)==0 && (y-y_trenutno)==0)
	{
		Stop();
		LCDClear();
		LCDWriteStringXY(0,0,"Nedefinisan atan");
		while(1);	
	}		
    
    ugao_temp = (int)(atan2((x - x_trenutno), (y - y_trenutno))/(2 * PI) * 360);
    
    return ugao_temp;
}

//funkcija za postavljanje ugla na nula
void setUgaoNula(void)
{

	long tmpT;
	
	setCommandInt('V',NORMALSPEED);
    setCommandInt('T', 0);
    do
    {
        tmpT = readUgaoInkrementi();
        if(tmpT > 56400)     
            break; 
    }while((tmpT < (- 500)) || (tmpT > 500));  //bilo 450

}

// funkcija za postavljanje ugla teta
char setUgao(int angle)
{
    static int previousAngle;
    int count;
    long prevAngleInc;
    long tmpT;
    long angleInc;
    long prevAngle;
	long ugao;
	
    
	setCommandInt('V',NORMALSPEED);
    if(firstTime) prevAngle = angle;
  
    if(prevAngle > 0)
    {
          prevAngle = angle;
        if (angle < 0 ) 
            angle += 360;
    }
    else
    {
          prevAngle = angle;
          if (angle <= 0) 
            angle += 360;
    }
	
	ugao = readUgaoStepeni();
    setCommandInt('T', angle);
    angleInc = (long)(((long)angle * PUNKRUG) / 360L);
    
    count = 0;
    do
    {
        _delay_ms(5);
		
		tmpT = readUgaoInkrementi();
        if (tmpT < 0) 
            tmpT += PUNKRUG;
        //zastita ako ne moze da dostigne ugao - vrati se u prethodni i okrene za 180 i pokusa da dodje u tacku u rikverc
        if((tmpT < (prevAngleInc + 50)) && (tmpT >(prevAngleInc - 50)))
        {
            if(++count == 77)   //zasto je 2500??
            {
				if(ugao == 0)
				{ 
					 setUgaoNula();
					 return 1;
				}				 
                else 
				{
                    setUgao(ugao);
					return 1;
				}	
			 		   					
              count = 0;
              ev_angleNotReached = 1; 
				
              break;
            }
        }  
		  
        prevAngleInc = tmpT;
            
        if(firstTime)
        {
            tmpT = PUNKRUG;
            firstTime = 0;
        }
        
        
    }while((tmpT < (angleInc - 450)) || (tmpT > (angleInc + 450)));  //bilo 450
    
    // za zastitu potrebno cuvati prethodni ugao
    if(!ev_angleNotReached) previousAngle = angle;
    _delay_ms(100);
    return 0;
}
	
extern void writeCoord(void);
extern char protocniDelay(unsigned long);
extern char protocniDelayZwei(unsigned long);

extern unsigned char counting;	
extern unsigned char countingZwei;	
				
// funkcija koja postavlja robota u zeljenju poziciju x, y, prvo se racuna ugao pa se okrene u taj ugao a onda se racuna distanca pa ide tamo
char gotoXY(int X_zeljeno, int Y_zeljeno, signed char direction, int speed, int precision, char detection)
{
    signed int x, previousX=0,zastita_X = 0;
    signed int y, previousY=0,zastita_Y = 0;
    int angle;
    long xl;
    long yl;
    int teta_zeljen;
    long d_zeljena_mm;
    short accel = 40;
	
    countingZwei = 0;    
    //racunanje i podesavanje ugla
    if( direction == 1 )
    {
        angle = racunanjeUgla(X_zeljeno, Y_zeljeno);
		/************************************************************************/
		/*	ako ne uspe da dostigne ugao, vrati se u prethodni i vraca da nije zavrsio kretnju, kao za sudar	                                                                   */
		/************************************************************************/
        if(angle == 0)		
        {    
			setUgaoNula();
		}		
        else 
		{
            if(setUgao(angle))
				return 1;			
		}   
		     
        /************************************************************************/
        /* prosla zastita, ako nije mogao da dostigne ugao pokusavao je da se okrece u drugu stranu*/
        /************************************************************************/
		/*if(ev_angleNotReached)
        {
            setUgao(angle+180);
            direction = -1;
            ev_angleNotReached = 0; 
         }  */                         
    }
    else
    {
        angle = racunanjeUgla(X_zeljeno, Y_zeljeno);
        if (angle == 180) 
            setUgaoNula();
        else
		{ 
            if(setUgao(angle+180))
				return 1;			
		}		
            
        
		/*if(ev_angleNotReached)
        {
            setUgao(angle);
            direction = 1;
            ev_angleNotReached = 0; 
         }   */ 
    }

    
    do
    {
        _delay_ms(5);
		
        x = readX();
        y = readY();
		
        xl = X_zeljeno - x;
        yl = Y_zeljeno - y;
    
        d_zeljena_mm = (long)( sqrt(( xl * xl ) + ( yl * yl )));
        d_zeljena_mm *= direction;
    
        if(absolut(d_zeljena_mm) < precision) 
            break; 
     
        setCommandInt('V', speed);
    
        //podesavanje ubrzanja
        setCommandShort('A', accel);
    
        // postavljanje distance
        setCommandInt('D', d_zeljena_mm);
	
		
		/************************************************************************/
		/*			TESTIRATI!!!                                                */
		/************************************************************************/
		
		if(protocniDelayZwei(1000))
		{
			
			if(((x < zastita_X + 2) && (x > zastita_X - 2)) && (y < zastita_Y + 2) && (y > zastita_Y - 2))
			{
				 Stop();
				 LCDClear();
				 LCDWriteStringXY(0,0,"COORD FAIL");
				 pauza_ms(50);
				 
				 moveOnDirection(120,direction*(-1),NORMALSPEED,ALL_DETECTIONS_OFF,0);
				 while(1);
				 return 1;
			}
			
			zastita_X = x;
			zastita_Y = y;	
			
			writeTime();
		}
		/*if(((x < previousX + 3) && (x > previousX - 3)) && (y < previousY + 3) && (y > previousY - 3))
        {
            if(++count == 22)   //1 sekund
            {		
//				LCDClear();
				//LCDWriteStringXY(0,0,"zapucan- gotoXY");
	
				if(direction == 1)  moveOnDirection(200,-1,NORMALSPEED,ALL_DETECTIONS_OFF,0);
                else                moveOnDirection(200,1,NORMALSPEED,ALL_DETECTIONS_OFF,0);
				PORTG = 0xFF;
				while(1);
                count = 0;
                break;
            }
        }
        */
        // korekcija ugla samo ako je robot na vecoj udaljenosti od 10 cm od zeljene tacke, ako je blize ne vrsi se korekcija ugla
        if( direction == 1)
        {
            if( d_zeljena_mm > 100 )
            {
                /************************************************************************/
				/*	DOBRO TESTIRATI!!!                                                  */
				/************************************************************************/
				
				if((xl)==0 && (yl)==0)
				{
					Stop();
					LCDClear();
					LCDWriteStringXY(0,0,"Nedefinisan atan");	
					while(1);
				}				
				
				teta_zeljen = (int)(atan2(xl, yl) / (2 * PI) * 360);
                setCommandInt('T', teta_zeljen);
            }
        }

        if( direction == -1)
        {
            if( d_zeljena_mm < -100)
            {
				/************************************************************************/
				/*	DOBRO TESTIRATI!!!                                                  */
				/************************************************************************/
				
				if((xl)==0 && (yl)==0)
				{
					Stop();
					LCDClear();
					LCDWriteStringXY(0,0,"Nedefinisan atan");	
					while(1);
				}	
				
				teta_zeljen = (int)(atan2(xl, yl) / (2 * PI) * 360);
				teta_zeljen += 180;
				setCommandInt('T', teta_zeljen);
            }
        }
	
        if(getDetections(detection,direction))
		{
			countingZwei = 0;
			Stop();
					
			return 1;
		}	
		
        previousX = x;
        previousY = y;
    
        x = readX();
        y = readY();
    
        xl = X_zeljeno - x;
        yl = Y_zeljeno - y;
    
        d_zeljena_mm = sqrt(( xl * xl ) + ( yl * yl ));
        d_zeljena_mm = d_zeljena_mm * direction;  
      writeTime();  
    }while( absolut(d_zeljena_mm) >  precision );
	
	countingZwei = 0;
	
    //kretnja odradjena do kraja - funkcija vraca 0
	return 0;
}

// fukcija za pomeranje po trenutnom pravcu, u zavisnosti od smera pomera se napred ili nazad za value


static char moveOnDirectionReal(int value, short direction,int speed, char detection, int breakTime)
{
    int x,y, X_zeljeno, Y_zeljeno, teta_zeljen,zastita_X,zastita_Y;
    long d_zeljena_mm;
    long xl;
    long yl;

    // racunanje nove tacke
    x = readX();
    y = readY();
    
    X_zeljeno = x + (direction)*(sin(readUgaoStepeni()* PI / 180) * value);
    Y_zeljeno = y + (direction)*(cos(readUgaoStepeni()* PI / 180) * value);
	
    counting = 0;
	countingZwei = 0;
	
    do
    {
        x = readX();
        y = readY();
        
        xl = X_zeljeno - x;
        yl = Y_zeljeno - y;
        
        d_zeljena_mm = sqrt(( xl * xl ) + ( yl * yl ));
        d_zeljena_mm *= direction;

        _delay_ms(5); //proveriti ovo!!!
        
        //podesavanje brzine 
		setCommandInt('V', speed);		
					
        // postavljanje distance
        setCommandInt('D', d_zeljena_mm);
        
        // korekcija ugla samo ako je robot na vecoj udaljenosti od 10 cm od zeljene tacke, ako je blize ne vrsi se korekcija ugla
        if(direction == 1)
        {
            if(d_zeljena_mm > 50)  //bilo 100
            {
                /************************************************************************/
				/*	DOBRO TESTIRATI!!!                                                  */
				/************************************************************************/
				
				if((xl)==0 && (yl)==0)
				{
					Stop();
					LCDClear();
					LCDWriteStringXY(0,0,"Nedefinisan atan");	
					while(1);
				}	
				
				teta_zeljen = (int)(atan2(xl, yl) / (2 * PI) * 360);
                setCommandInt('T', teta_zeljen);
            }
        }

        if( direction == -1)
        {
            if( d_zeljena_mm < -50) //200
            {
                /************************************************************************/
				/*	DOBRO TESTIRATI!!!                                                  */
				/************************************************************************/
				
				if((xl)==0 && (yl)==0)
				{
					Stop();
					LCDClear();
					LCDWriteStringXY(0,0,"Nedefinisan atan");
					while(1);	
				}	
				
				teta_zeljen = (int)(atan2(xl, yl) / (2 * PI) * 360);
                teta_zeljen += 180;
                setCommandInt('T', teta_zeljen);
            }
        }
	
		if(breakTime>0)
		{
			if(protocniDelay(breakTime))
			{
				Stop();
				
				return 0;
			}			
		}
		
		if(getDetections(detection,direction))
		{
			Stop();
			
			counting = 0;
			
			return 1;
		}
		
		if(protocniDelayZwei(2500))
		{
			
			if(((x < zastita_X + 2) && (x > zastita_X - 2)) && (y < zastita_Y + 2) && (y > zastita_Y - 2))
			{
				 Stop();
				 
				 pauza_ms(50);
				 
				 moveOnDirection(200,-direction,NORMALSPEED,ALL_DETECTIONS_OFF,0);
				 while(1);
				
				return 1;
			}
			
			
			zastita_X = x;
			zastita_Y = y;	
		}
				
	writeTime();
    }while(absolut(d_zeljena_mm) >  NORMALPRECISION );
	
	counting = 0;
	countingZwei = 0;

	return 0;
}

char moveOnDirection(int value,short direction,int speed,char detection,int breakTime)
{
	int X_pocetno,Y_pocetno,x,y,predjeni_put=0;
	char collisionCounter = 0;
	
	X_pocetno = readX();
	Y_pocetno = readY();
	
	while(moveOnDirectionReal(value,direction,speed,detection,breakTime))
	{			

		_delay_ms(20);
			
		moveOnDirection(10,direction*(-1),LOWSPEED,ALL_DETECTIONS_OFF,0);
		_delay_ms(1000);
	
		x = readX();
		y = readY();
		
		predjeni_put = sqrt(pow((x-X_pocetno),2) + pow((y-Y_pocetno),2));
		
		X_pocetno = x;
		Y_pocetno = y;
			
		value -= predjeni_put;
		
		if(++collisionCounter>=15)	//ako 10 puta vec pokusava da dostigne poziciju izlazi i vraca 1, onda je protivnicko ili nase malo govno zabagovano ispred
			return 1;			
	}
	
	return 0;
	
}