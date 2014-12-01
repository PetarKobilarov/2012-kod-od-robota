/**********************************************************************************************************************************************
Biblioteka USART modula
USART0: komunikacija sa pogonskim drajverom
USART1: komunikacija sa aktuatorima
***********************************************************************************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"


#if defined (__AVR_ATmega128__)

/****************************************************************************************************************************************
Inicijalizacija USART0 modula:
-prijemnik, predajnik
-asinhroni
-1 stop bit
-bez paritet bita
****************************************************************************************************************************************/
void initUart0(void)
{	
	UCSR0A=0;
	UCSR0B=(1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	UCSR0C=(1<<UCSZ01) | (1<<UCSZ00);

	//UBRR0H=(unsigned char) (UART0_UBRR>>8);
	//UBRR0L=(unsigned char)  UART0_UBRR;	
	UBRR0H=0;
	UBRR0L=11;
	
	SREG |= 0x80;
}


#define TX0_BUFFER_SIZE 	64
static volatile unsigned char tx0_buffer[TX0_BUFFER_SIZE];
static volatile unsigned char tx0_wr_index=0;
static volatile unsigned char tx0_counter=0;
static volatile unsigned char tx0_rd_index=0;

/****************************************************************************************************************************************
Transmit interrupt USART0 modula
****************************************************************************************************************************************/

ISR(USART0_TX_vect)
{
	if(tx0_counter)
	{
		tx0_counter--;
		UDR0=tx0_buffer[tx0_rd_index];
		if(++tx0_rd_index == TX0_BUFFER_SIZE)
			tx0_rd_index = 0;	
	}	
}

/****************************************************************************************************************************************
Slanje podataka USART0 modulom
****************************************************************************************************************************************/

void UART0_Write(unsigned char data)
{
	while(tx0_counter == TX0_BUFFER_SIZE);
	
	cli();
	
	if(tx0_counter || (UCSR0A & 0x20) == 0)
	{
		tx0_buffer[tx0_wr_index] = data;
		if(++tx0_wr_index == TX0_BUFFER_SIZE)	
			tx0_wr_index = 0;
			
		tx0_counter++;		
	}else{
		UDR0=data;
	}	
	sei();
}

#define RX0_BUFFER_SIZE 	64
static volatile unsigned char rx0_buffer[RX0_BUFFER_SIZE];
static volatile unsigned char rx0_wr_index = 0;
static volatile unsigned char rx0_counter = 0;
static volatile unsigned char rx0_rd_index = 0;

/****************************************************************************************************************************************
Receive interrupt USART0 modula
****************************************************************************************************************************************/

ISR(USART0_RX_vect)
{
	unsigned char status;
	status=UCSR0A;

	if(!(status & (FRAME_ERROR | DATA_OVERRUN | PARITY_ERROR)))	
	{
		rx0_buffer[rx0_wr_index] = UDR0;
		if(++rx0_wr_index == RX0_BUFFER_SIZE)
			rx0_wr_index = 0;
			
		rx0_counter++;
	}
}

/****************************************************************************************************************************************
Primanje podataka USART0 modulom
****************************************************************************************************************************************/

unsigned char UART0_Read(void)
{
	unsigned char data;
	
	while(rx0_counter == 0);
	
	data=rx0_buffer[rx0_rd_index];
	
	if(++rx0_rd_index == RX0_BUFFER_SIZE)
		rx0_rd_index = 0;
		
	rx0_counter--;
	
	return data;
}

/****************************************************************************************************************************************
Inicijalizacija USART1 modula:
-predajnik
-asinhroni
-1 stop bit
-bez paritet bita
****************************************************************************************************************************************/
void initUart1(void)
{	
	UCSR1A=0;
	UCSR1B=(1<<TXCIE1) | (1<<TXEN1);
	UCSR1C=(1<<UCSZ10) | (1<<UCSZ11);
	
	UBRR1H=UBRR1_VALUE>>8;
	UBRR1L=UBRR1_VALUE; //ako bude kenjao staviti za L 71 decimalno, H ide 0

	//UBRR1H=(unsigned char) (UART1_UBRR>>8);
	//UBRR1L=(unsigned char)  UART1_UBRR;
	
	SREG |= 0x80;	
}


#define TX1_BUFFER_SIZE 	64
static volatile unsigned char tx1_buffer[TX1_BUFFER_SIZE];
static volatile unsigned char tx1_wr_index=0;
static volatile unsigned char tx1_counter=0;
static volatile unsigned char tx1_rd_index=0;

/****************************************************************************************************************************************
Transmit interrupt USART1 modula
****************************************************************************************************************************************/
//#error "Linker error- idiotic programming"
ISR(USART1_TX_vect)
{
	if(tx1_counter)
	{
		tx1_counter--;
		UDR1=tx1_buffer[tx1_rd_index];
		if(++tx1_rd_index == TX1_BUFFER_SIZE)
			tx1_rd_index = 0;	
	}	
}

void altUart0Write(unsigned char data)
{
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0=data;	
}

/*void stopKretanje(void)
{
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR1='S';
}*/
void altUart1Write(unsigned char data)
{
	while ( !( UCSR1A & (1<<UDRE1)) );
	UDR1=data;
}
/****************************************************************************************************************************************
Slanje podataka USART1 modulom
**

**************************************************************************************************************************************/

void UART1_Write(unsigned char data)
{
	while(tx1_counter == TX1_BUFFER_SIZE);
	
	cli();
	
	if(tx1_counter || (UCSR1A & 0x20) == 0)
	{
		tx1_buffer[tx1_wr_index] = data;
		
		if(++tx1_wr_index == TX1_BUFFER_SIZE)	
			tx1_wr_index = 0;
			
		tx1_counter++;		
	}else{
		UDR1=data;
	}
		
	sei();
}

#elif defined (__AVR_ATmega32__)


void initUart(void)
{

	UCSRA=0;
	UCSRB=(1<<RXCIE) | (1<<TXCIE) | (1<<RXEN) | (1<<TXEN);
	UCSRC=(1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
	
	UBRRH=(unsigned char)(UART_UBRR>>8);
	UBRRL=(unsigned char) UART_UBRR;
	
	SREG |= 0x80;
}

#define TX_BUFFER_SIZE 	64
static volatile unsigned char tx_buffer[TX_BUFFER_SIZE];
static volatile unsigned char tx_wr_index=0;
static volatile unsigned char tx_counter=0;
static volatile unsigned char tx_rd_index=0;


ISR(USART_TXC_vect)
{
	if(tx_counter)
	{
		tx_counter--;
		UDR=tx_buffer[tx_rd_index];
		if(++tx_rd_index==TX_BUFFER_SIZE)
			tx_rd_index=0;		
	}	
}

void UART_Write(unsigned char data)
{
	
	while(tx_counter==TX_BUFFER_SIZE);
	cli();
	if(tx_counter || (UCSRA & 0x20)==0)
	{
		tx_buffer[tx_wr_index]=data;
		if(++tx_wr_index==TX_BUFFER_SIZE)	tx_wr_index=0;
			
		tx_counter++;		
	}else{
		UDR=data;
	}	
	sei();
}

#define RX_BUFFER_SIZE 	64
static volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
static volatile unsigned char rx_wr_index = 0;
static volatile unsigned char rx_counter = 0;
static volatile unsigned char rx_rd_index = 0;

/****************************************************************************************************************************************
Receive interrupt USART0 modula
****************************************************************************************************************************************/

ISR(USART_RXC_vect)
{
	unsigned char status;
	status=UCSRA;

	if(!(status & (FRAME_ERROR | DATA_OVERRUN | PARITY_ERROR)))	
	{
		rx_buffer[rx_wr_index] = UDR;
		if(++rx_wr_index == RX_BUFFER_SIZE)
			rx_wr_index = 0;
			
		rx_counter++;
	}
}

unsigned char UART_Read(void)
{
	unsigned char data;
	
	while(rx_counter == 0);
	
	data=rx_buffer[rx_rd_index];
	
	if(++rx_rd_index == RX_BUFFER_SIZE)
		rx_rd_index = 0;
		
	rx_counter--;
	
	return data;
}

#endif
