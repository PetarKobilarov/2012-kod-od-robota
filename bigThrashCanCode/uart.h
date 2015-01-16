#ifndef _USART_H_INCLUDED
#define _USART_H_INCLUDED

#ifndef F_CPU
#define F_CPU 11059200ULL
#endif

#define UART_ISR_ON		1
#define UART_ISR_OFF	0

/*********************************************************************************
* Function Name		: UARTx_Init
* Description           : Inicijalizuje UART modul kao predajnik i kao prijemnik
* Parameters           : unsigned int baud
					  	  	      char isrOnOff - da li da se koristi rx i tx interrupt
* Return Value        : void
*********************************************************************************/
void UART0_Init(unsigned int baud, char use_interrupt);
void UART1_Init(unsigned int baud, char use_interrupt);

/*********************************************************************************
* Function Name		: UARTx_Write
* Description           : Pisanje bajta na UART modul.
* Parameters           : unsigned char data
* Return Value        : void
*********************************************************************************/
void UART0_Write(unsigned char data);
void UART1_Write(unsigned char data);

/*********************************************************************************
* Function Name		: UARTx_Read
* Description           : Citanje primljenih bajtova sa UART modula.
* 							      Bajtovi se smestaju u FIFO red(samo ako se koristi ISR),
* 							      ova funkcija preuzima bajt po bajt iz reda
* Parameters           : none
* Return Value        : unsigned char
*********************************************************************************/
unsigned char UART0_Read(void);
unsigned char UART1_Read(void);

/*********************************************************************************
* Function Name		: UARTx_CheckRx
* Description           : Funkcija vraca broj neprocitanih bajtova iz FIFO reda
* 								   za modul- samo ako se koristi ISR
* Parameters           : none
* Return Value        : unsigned char numOfBytes
*********************************************************************************/
unsigned char UART0_CheckRx(void);
unsigned char UART1_CheckRx(void);

/*********************************************************************************
* Function Name		: UARTx_ClearXX
* Description           : Funkcija odbacuje sadrzaj odgovarajucih bafera-
*                                 ima smisla samo uz ISR
* Parameters           : none
* Return Value        : void
*********************************************************************************/
void UART0_ClearRx(void);
void UART0_ClearTx(void);
void UART1_ClearRx(void);
void UART1_ClearTx(void);

/*********************************************************************************
* Function Name		: UARTx_Write_Clean
* Description           : Funkcija vrsi upis bajta na prenosnu liniju bez koriscenja ISR- a
* Parameters           : unsigned char data
* Return Value        : void
*********************************************************************************/
void UART0_Write_Clean(unsigned char data);
void UART1_Write_Clean(unsigned char data);

/*********************************************************************************
* Function Name		: UART_WriteString
* Description           : Funkcija za pisanje stringa- niza karaktera koji se
* 								   zavrsavaju sa '\0' na prenosnu liniju
* Parameters           : char *tx_buffer - string za prenos
*                                 void(*UART_Write_func)(unsigned char data) - pokazivac na f- ju upisa na prenosnu liniju.
*                                  -> mora biti tog oblika
* Return Value        : void
*********************************************************************************/
void UART_WriteString(char *tx_buffer, void (*UART_Write_func)(unsigned char data));

/*********************************************************************************
* Function Name		: UART_ReadString
* Description           : Funkcija za citanje stringa sa prenosne linije, do delimitera
* Parameters           : char *tx_buffer - bafer mora biti dovoljno velik
*                                 unsigned char (*UART_Read_func)(void) - pokazivac na f- ju citanja sa prenosne linije.
*                                  -> mora biti tog oblika
* Return Value        : void
*********************************************************************************/
void UART_ReadString(char *rx_buffer, char delimiter, unsigned char (*UART_Read_func)(void));

/*********************************************************************************
* Function Name		: UART_WriteInt
* Description           : Funkcija za prenos int- a u ASCII obliku
* Parameters           : int num
*                                void(*UART_Write_func)(unsigned char data) - pokazivac na f- ju upisa na prenosnu liniju.
*                                  -> mora biti tog oblika
* Return Value        : void
*********************************************************************************/
void UART_WriteInt(int num, void (*UART_WriteFunc)(unsigned char));

#endif
