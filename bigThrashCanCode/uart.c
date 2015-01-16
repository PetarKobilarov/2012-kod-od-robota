/**********************************************************************************
* File Name			: usart.c
* Description       : Implamentacija funkcija za upravljanje UART modulom na ATMEGA128/AT90CAN128
**********************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#define FRAME_ERROR		(1<<4)
#define DATA_OVERRUN	(1<<3)
#define PARITY_ERROR	(1<<2)


#define TX0_BUFFER_SIZE 	20
static volatile unsigned char tx0_buffer[TX0_BUFFER_SIZE];
static volatile unsigned char tx0_wr_index;
static volatile unsigned char tx0_counter;
static volatile unsigned char tx0_rd_index;

#define RX0_BUFFER_SIZE 	20
static volatile unsigned char rx0_buffer[RX0_BUFFER_SIZE];
static volatile unsigned char rx0_wr_index;
static volatile unsigned char rx0_counter;
static volatile unsigned char rx0_rd_index;


/*********************************************************************************
* Function Name		: initUart0
* Description           : Inicijalizuje UART0 modul kao predajnik i kao prijemnik
* Parameters           : unsigned int baud
					  	  	      char isrOnOff - da li da se koristi rx i tx interrupt
* Return Value        : void
*********************************************************************************/
void UART0_Init(unsigned int baud, char isrOnOff)
{
	unsigned int tempBaud =(double) F_CPU/(16.0 * baud) - 0.5;

	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	if(isrOnOff == UART_ISR_ON)
	{
		UCSR0B |= ((1 << RXCIE0) | (1 << TXCIE0));
		SREG |= 0x80;
	}

	UBRR0H = tempBaud >> 8;
	UBRR0L = tempBaud;

	rx0_wr_index = rx0_rd_index = rx0_counter = 0;
	tx0_wr_index = tx0_rd_index = tx0_counter = 0;
}

/*********************************************************************************
* Function Name		: ISR
* Description           : Obradjivac prekida za UART0 predajni modul
*********************************************************************************/
ISR(USART0_TX_vect)
{
	if(tx0_counter)
	{
		tx0_counter--;
		UDR0 = tx0_buffer[tx0_rd_index];

		if(++tx0_rd_index == TX0_BUFFER_SIZE)
			tx0_rd_index = 0;
	}
}

/*********************************************************************************
* Function Name		: UART0_Write
* Description           : Slanje bajta preko UART0 modula
* Parameters           : unsigned int data
* Return Value        : void
*********************************************************************************/
void UART0_Write(unsigned char data)
{
	if( (UCSR0B >> TXCIE0) & 0x01)
	{
		while(tx0_counter == TX0_BUFFER_SIZE);

		cli();

		if(tx0_counter || (UCSR0A & 0x20) == 0)
		{
			tx0_buffer[tx0_wr_index] = data;
			if(++tx0_wr_index == TX0_BUFFER_SIZE)
				tx0_wr_index = 0;

			tx0_counter++;
		}
		else
			UDR0 = data;
		sei();
	}
	else
		UART0_Write_Clean(data);
}

/*********************************************************************************
* Function Name		: ISR
* Description           : Obradjivac prekida za UART0 prijemni modul
*********************************************************************************/
ISR(USART0_RX_vect)
{
	unsigned char status;
	status = UCSR0A;

	if(!(status & (FRAME_ERROR | DATA_OVERRUN | PARITY_ERROR)))
	{
		rx0_buffer[rx0_wr_index] = UDR0;
		if(++rx0_wr_index == RX0_BUFFER_SIZE)
			rx0_wr_index = 0;

		rx0_counter++;
	}
}

/*********************************************************************************
* Function Name		: UART0_Read
* Description           : Citanje primljenih bajtova sa UART0 modula.
* 							      Bajtovi se smestaju u FIFO red, ova funkcija preuzima bajt po bajt iz reda
* Parameters           : none
* Return Value        : unsigned char
*********************************************************************************/
unsigned char UART0_Read(void)
{
	unsigned char data;

	if((UCSR0B >> RXCIE0) & 0x01)
	{
		while(rx0_counter == 0);

		cli();

		data = rx0_buffer[rx0_rd_index];

		if(++rx0_rd_index == RX0_BUFFER_SIZE)
			rx0_rd_index = 0;

		rx0_counter--;

		sei();
	}
	else
	{
		while(!(UCSR0A >> RXC0));
		data = UDR0;
	}

	return data;
}

/*********************************************************************************
* Function Name		: UART0_CheckRx
* Description           : Funkcija vraca broj neprocitanih bajtova iz FIFO reda
* 								   za modul- samo ako se koristi ISR
* Parameters           : none
* Return Value        : unsigned char numOfBytes
*********************************************************************************/
unsigned char UART0_CheckRx(void)
{
	return rx0_counter;
}

void UART0_ClearRx(void)
{
	rx0_counter = rx0_rd_index = rx0_wr_index = 0;
}

void UART0_ClearTx(void)
{
	tx0_counter = tx0_rd_index = tx0_wr_index = 0;
}


#define TX1_BUFFER_SIZE 	20
static volatile unsigned char tx1_buffer[TX1_BUFFER_SIZE];
static volatile unsigned char tx1_wr_index;
static volatile unsigned char tx1_counter;
static volatile unsigned char tx1_rd_index;

#define RX1_BUFFER_SIZE 	20
static volatile unsigned char rx1_buffer[RX1_BUFFER_SIZE];
static volatile unsigned char rx1_wr_index;
static volatile unsigned char rx1_counter;
static volatile unsigned char rx1_rd_index;


/*********************************************************************************
* Function Name		: initUart1
* Description           : Inicijalizuje UART1 modul kao predajnik i kao prijemnik
* Parameters           : unsigned int baud
					  	  	      char isrOnOff - da li da se koristi rx i tx interrupt
* Return Value        : void
*********************************************************************************/
void UART1_Init(unsigned int baud, char isrOnOff)
{
	unsigned int tempBaud = (double)F_CPU/(16.0 * baud) - 0.5;

	UCSR1A = 0;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);

	if(isrOnOff == UART_ISR_ON)
	{
		UCSR1B |= (1 << RXCIE1) | (1 << TXCIE1);
		SREG |= 0x80;
	}

	UBRR1H = tempBaud >> 8;
	UBRR1L = tempBaud;

	tx1_wr_index = tx1_rd_index = tx1_counter = 0;
	rx1_rd_index = rx1_wr_index = rx1_counter = 0;
}


/*********************************************************************************
* Function Name		: ISR
* Description           : Obradjivac prekida za UART1 predajni modul
*********************************************************************************/
ISR(USART1_TX_vect)
{
	if(tx1_counter)
	{
		tx1_counter--;
		UDR1 = tx1_buffer[tx1_rd_index];
		if(++tx1_rd_index == TX1_BUFFER_SIZE)
			tx1_rd_index = 0;
	}
}

/*********************************************************************************
* Function Name		: ISR
* Description           : Obradjivac prekida za UART1 prijemni modul
*********************************************************************************/
ISR(USART1_RX_vect)
{
	unsigned char status;
	status = UCSR1A;

	if(!(status & (FRAME_ERROR | DATA_OVERRUN | PARITY_ERROR)))
	{
		rx1_buffer[rx1_wr_index] = UDR1;
		if(++rx1_wr_index == RX1_BUFFER_SIZE)
			rx1_wr_index = 0;

		rx1_counter++;
	}
}

void UART0_Write_Clean(unsigned char data)
{
	while ( !(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void UART1_Write_Clean(unsigned char data)
{
	while ( !( UCSR1A & (1<<UDRE1)) );
	UDR1 = data;
}
/****************************************************************************************************************************************
Slanje podataka USART1 modulom
**

**************************************************************************************************************************************/

void UART1_Write(unsigned char data)
{
	if( (UCSR1B >> TXCIE1) & 0x01)
	{
		while(tx1_counter == TX1_BUFFER_SIZE);

		cli();

		if(tx1_counter || (UCSR1A & 0x20) == 0)
		{
			tx1_buffer[tx1_wr_index] = data;
			if(++tx1_wr_index == TX1_BUFFER_SIZE)
				tx1_wr_index = 0;

			tx1_counter++;
		}
		else
			UDR1 = data;

		sei();
	}
	else
		UART1_Write_Clean(data);
}

unsigned char UART1_Read(void)
{
	unsigned char data;

	if((UCSR1B >> RXCIE1) & 0x01)
	{
		while(rx1_counter == 0);

		cli();

		data = rx1_buffer[rx1_rd_index];

		if(++rx1_rd_index == RX1_BUFFER_SIZE)
			rx1_rd_index = 0;

		rx1_counter--;

		sei();
	}
	else
	{
		while(!(UCSR1A >> RXC1));
		data = UDR1;
	}

	return data;
}

unsigned char UART1_CheckRx(void)
{
	return rx1_counter;
}

void UART1_ClearRx(void)
{
	rx1_counter = rx1_rd_index = rx1_wr_index = 0;
}

void UART1_ClearTx(void)
{
	tx1_counter = tx1_rd_index = tx1_wr_index = 0;
}

void UART_WriteString(char *text, void (*UART_Write_func)(unsigned char))
{
	unsigned char i = 0;

	while(*(text + i) != '\0')
		UART_Write_func(*(text + (i++)));
}

void UART_ReadString(char *buffer, char delimiter, unsigned char (*UART_Read_func)(void))
{
	unsigned char i = 0, temp;

	do
	{
		temp = UART_Read_func();
		*(buffer + (i++)) = temp;
	}while(temp != delimiter);
}

void UART_WriteInt(int num, void (*UART_WriteFunc)(unsigned char))
{
	unsigned char factors[5];
	char i = 0;

	if(num < 0)
	{
		UART_WriteFunc('-');
		num *= -1;
	}

	do
	{
		factors[i++] = num % 10;
		num /= 10;
	}while(num != 0);

	do
	{
		UART_WriteFunc(factors[--i] + '0');
	}while(i > 0);
}
