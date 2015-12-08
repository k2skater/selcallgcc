/****************************************************
TARGET: 		ATMEGA8
CLOCK:			8 MHz
Peripherals:	CMX823 Programmable Paging Tone Decoder
Author:			Alexandros Skafidas
Date:			17 Feb 2008
Compiler:		AVR-GCC (WinAVR-GCC)
Flash Usage:	34.8%
RAM Usage:		%

			(RESET) 		PC6	1  28 PC5 (ADC5/SCL) 	------>	LCD RS
			(RXD) 			PD0	2  27 PC4 (ADC4/SDA) 	------>	LCD RS
			(TXD) 			PD1	3  26 PC3 (ADC3)		------>	LCD D7
IRQN ----->	(INT0)			PD2	4  25 PC2 (ADC2)		------>	LCD D6
switch --->	(INT1)			PD3	5  24 PC1 (ADC1)		------>	LCD D5
			(XCK/T0)		PD4	6  23 PC0 (ADC0)		------>	LCD D4
			VCC 				7  22 GND
			GND					8  21 AREF
			(XTAL1/TOSC1) 	PB6 9  20 AVCC
			(XTAL2/TOSC2)	PB7	10 19 PB5 (SCK)			------> SERIAL CLOCK
			(T1)			PD5	11 18 PB4 (MISO)		<------ REPLY DATA
			(AIN0)			PD6	12 17 PB3 (MOSI/OC2)	------> COMMAND DATA
			(AIN1)			PD7	13 16 PB2 (SS/OC1B)		------> CHIP SELECT
LED0		(ICP1)			PB0	14 15 PB1 (OC1A)		LED1

LED0 - interrupt 0
LED1 - sucessfull decode

HISTORY:
17 Feb 08	-	First stable version

TODO:
Adjustable bandwidth
*****************************************************/

#ifndef 	F_CPU
#define 	F_CPU 			8000000UL
#endif
// Define baud rate

#define USART_BAUD 38400

#define USART_UBBR_VALUE ((F_CPU/(USART_BAUD<<4))-1)

#include <stdbool.h>
#include <stdio.h>
#include <string.h>						//strlen
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd_lib.h"

#define		TRUE						1
#define		FALSE						0
#define		BIT(x)						(1 << (x))

#define		NUMBER_OF_TONES				15
#define		TOLERANCE					3
#define		MINTONES					5
//#define	BANDWIDTH

#define		CMX_GENERAL_RESET			0x01
#define		CMX_CONTROL					0x30
#define		CMX_AUX_CONTROL				0x32
#define		CMX_GENERAL					0x33
#define		CMX_TONE_PARAMETERS			0x34
#define		CMX_DECODED_TONE			0x38
#define		CMX_DECODED_PARM			0x3C
#define		CMX_STATUS					0x3F

//CMX status bits (3 and 5 cleared when STATUS read)
#define		RAM1OR2						(1<<0)
#define		UNLISTED_TONE				(1<<1)
#define		TONE_DECODE					(1<<2)
#define		DECODE_STATUS_CHANGE		(1<<3)
#define		RAM_FULL					(1<<4)
#define		TONE_CHANGE					(1<<5)

#define 	TONESETS					5

#define 	TONESET_CCIR				0
#define 	TONESET_ZVEI				1
#define 	TONESET_EEA					2
#define 	TONESET_EIA					3
#define 	TONESET_NATEL				4

//prototypes
void Spi_Init(void);
void Cmx823_Init(void);
void Port_Init(void);
void LCDinit(void);
void LCDclr(void);
void LCDcursorOnBlink(void);
void LCDstring(uint8_t*, uint8_t);
void LCDGotoXY(uint8_t, uint8_t);
char hex2ascii(char hex);
void LCDsendChar(uint8_t);
void Spi_TxRx(uint8_t* receiveArray, uint8_t numberbytes, ...);
void statuscheck(void);
void TimerInit(void);
void USART_Init(void);
static int put_char(char c, FILE *stream);

uint8_t lasttone=0x0E,aquiredtones=0,ms10=0,max=0,min=255,toneset=TONESET_CCIR;

char 	decodedsequence[LCD_LINE_LENGTH] = {0};		//defined in lcd.h
uint8_t durationarray[LCD_LINE_LENGTH] = {0};

uint16_t CCIRtones2 [NUMBER_OF_TONES][TONESETS]={
{0x100D,0x138A,0x100D,0x04B0,0x0330},	//0
{0x0911,0x0891,0x0911,0x060C,0x0598},	//1
{0x0996,0x0986,0x0996,0x0697,0x061E},	//2
{0x0A84,0x0A82,0x0A84,0x081C,0x0705},	//3
{0x0B0C,0x0B85,0x0B0C,0x0988,0x0794},	//4
{0x0C00,0x0C88,0x0C00,0x0A8F,0x0A02},	//5
{0x0C8B,0x0D8D,0x0C8B,0x0C00,0x0B03},	//6
{0x0D84,0x0F06,0x0D84,0x0D06,0x0C0A},	//7
{0x0E80,0x1083,0x0E80,0x0E0C,0x0D82},	//8
{0x0F0E,0x1207,0x0F0E,0x0F80,0x0E8E},	//9
{0x138A,0x1705,0x088E,0x118A,0x0F88},	//A
{0x078E,0x0690,0x078E,0x1405,0x1083},	//B
{0x1284,0x0803,0x1284,0x1085,0x110C},	//C
{0x080D,0x0718,0x080D,0x1300,0x1207},	//D
{0x1180,0x1582,0x1180,0x03A8,0x1302}};  //E

char * Toneset[TONESETS]={"CCIR","ZVEI","EEA","EIA","NATEL"};

static int put_char(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(put_char,NULL,_FDEV_SETUP_WRITE);

int main(void)
{
	SREG &= ~BIT(7);				// Disable all interrupts

	Port_Init();
	stdout = &mystdout;		//set the output stream

	GICR = 0x00;					// disable external INTs, clear before changing ISC10,11
	MCUCR |= 0x00;		
	GICR  |= BIT(INT0) |BIT(INT1);	// enable external INTs

	LCDinit();
	LCDclr();
	USART_Init();

	//LCDcursorOnBlink();

	TimerInit();
	Spi_Init();
	Cmx823_Init();
	GIFR |= BIT(INTF0);
	SREG |= BIT(7);					// Global Interrupt enable
	puts("check1");
	printf("check2");

	while (TRUE)
	{
	_delay_ms(500);
	puts("check3");

	}
}

// IRQN received from CMX823 when going from 1 to 0 - change in decode status
//Aquire the minimum required tones and THEN display on LCD
ISR(INT1_vect){
	LCDclr();
	statuscheck();
	_delay_ms(100);
	GIFR |= BIT(INTF1);				// have to clear flag (cleared auto/ly ?? )
}

ISR(INT0_vect){
	uint8_t decodedtone,asciitone,spitemp,i;
	uint8_t DataArray[2] = {0x00,0x00};


	Spi_TxRx(DataArray,2,CMX_STATUS,0x00);				//read STATUS to see what happened (read action clears CMX irq)
	spitemp=DataArray[1];
	
	PORTB ^= BIT(0);
	if ((spitemp & TONE_DECODE)== TONE_DECODE){			// check for TONE CHANGE as well !!!!

		Spi_TxRx(DataArray,2,CMX_DECODED_TONE,0x00);	//write dummy byte to get 1 byte back
		decodedtone = DataArray[1];						//lower 5 bits

		if (decodedtone == lasttone)
			goto RTI;

		TCNT1=0;
		ms10=0;

		if (decodedtone == 0x0E)
			decodedtone = lasttone;
		lasttone=decodedtone;

		if ((decodedtone > 0x09) && (decodedtone <= 0x0F))
	   		asciitone = decodedtone + 0x37;
		else
			asciitone = decodedtone + 0x30;

		if(aquiredtones <= LCD_LINE_LENGTH)
		    decodedsequence[aquiredtones]=asciitone;

		aquiredtones++;
	}else{	//NOTONE
		// After the CMX823 has deresponded into the
		//NOTONE state, the internal decoded data history should be cleared by resetting
		//the GENERAL Register $33 bits 6 and 7 to �0� for a short period (> 10�s). These
		//bits should then be returned to their previous values. This will ensure that the
		//decoding of a new tone is not influenced by assessments made on the previous tone.
		//TIMSK &=  ~_BV(OCIE1A);					// Compare Match interrupt is DISabled	
		Spi_TxRx(DataArray,2,CMX_GENERAL,0x00);		//reset after NOTONE
		_delay_us(20);	
		Spi_TxRx(DataArray,2,CMX_GENERAL,0x40);		//enable 1st tone group decoding

		if(aquiredtones >= MINTONES){
			uint8_t temp;
			for(i=2;i<aquiredtones;i++){			//1-not valid, 2-ignore 1st tone
				temp=durationarray[i];
				if(temp<min)
					min = temp;
				if(temp>max)
					max = temp;
					//LCDGotoXY(i*2,1);	
					//LCDsendChar((hex2ascii(durationarray[i]>>4)));
					//LCDsendChar((hex2ascii(durationarray[i])));
			}
			if ((max-min)<=TOLERANCE){				//valid sequence!!!
				LCDclr();				
				put_char(0x0A,&mystdout);
				put_char(0x0D,&mystdout);
				PORTB ^= BIT(1);
				LCDGotoXY(0,0);
				for(i=0;i<aquiredtones;i++){
					LCDsendChar(decodedsequence[i]);
					put_char(decodedsequence[i],&mystdout);
				}
			}
		}
		lasttone=0x61;								//use an invalid tone
		aquiredtones=0;
		max=0;
		min=255;
	}
RTI:
	GIFR |= BIT(INTF0);								// have to clear flag (cleared auto/ly ?? )
}

ISR(TIMER1_COMPA_vect){
	ms10++;
	durationarray[aquiredtones]=ms10;
	TCNT1 =  0;
}

void Cmx823_Init(){
	uint8_t tone,highbyte,lowbyte;
	uint16_t twobytes;
	uint8_t DataArray[3] = {0x00,0x00,0x00};
	Spi_TxRx(DataArray,1,CMX_GENERAL_RESET); 		//zero power mode, clear all registers, but retain RAM contents
	Spi_TxRx(DataArray,2,CMX_CONTROL,0x01);			//RAM1, clear mode
	Spi_TxRx(DataArray,2,CMX_CONTROL,0x81);			//RAM2, clear mode
	Spi_TxRx(DataArray,2,CMX_AUX_CONTROL,0x01);		//Bit 0 of the AUXILIARY CONTROL Register $32 should be set to �1� during this process
	LCDclr();
	LCDstring((uint8_t*)Toneset[toneset],strlen(Toneset[toneset]));
	LCDGotoXY(0,1);
	for (tone=0;tone<NUMBER_OF_TONES;tone++){
       	twobytes = CCIRtones2[tone][toneset];
		highbyte = ((twobytes & 0xFF00)>>8);
		lowbyte  = (twobytes & 0x00FF);		
		highbyte |= 0x40;							//for 5/6 tones set 1st and 2nd tone group (0x40 = 2nd group????)

		LCDsendChar('*');
		Spi_TxRx(DataArray,3,CMX_TONE_PARAMETERS,highbyte,lowbyte);
		_delay_ms(100);
	}
	Spi_TxRx(DataArray,2,CMX_AUX_CONTROL,0x00);		//then reset to 0 when it is required to decode an input signal
	//Spi_TxRx(DataArray,2,CMX_CONTROL,0xC6);		//RAM1, Fast mode, BW=0.3%, normal mode
	Spi_TxRx(DataArray,2,CMX_CONTROL,0xCA);			//RAM1, Fast mode, BW=0.5%, normal mode
	//Spi_TxRx(DataArray,2,CMX_CONTROL,0xD2);		//RAM1, Fast mode, BW=0.9%, normal mode
	Spi_TxRx(DataArray,2,CMX_GENERAL,0x40);			//enable 1st tone group decoding - interrupt enable
	_delay_ms(30);									//stabilize
	LCDGotoXY(0,0);
}

char hex2ascii(char hex)
{
	char lcdchar;
		lcdchar = (hex&0x0F);
		if (lcdchar > 9){
     		switch(lcdchar){
			case 0x0A: lcdchar = 'A';break;
			case 0x0B: lcdchar = 'B';break;
			case 0x0C: lcdchar = 'C';break;
			case 0x0D: lcdchar = 'D';break;
			case 0x0E: lcdchar = 'E';break;
			case 0x0F: lcdchar = 'F';break;
			}
		}
		else
		  lcdchar = lcdchar + 0x30;
     return lcdchar;
}

void Port_Init(void)
{
	PORTB = 0x00; DDRB = 0xEF;	//8		PB4 input for MISO
	PORTC = 0x00; DDRC = 0xEF;	//7
	PORTD = 0x0C; DDRD = 0xF3;	//8		PD2,P3 input for INT0,INT1 - enable pullups
}

void Spi_Init() {
	unsigned char tmp;

	DDRB |= _BV(PB3) | _BV(PB5) | _BV(PB2);		// set MOSI and SCK and SS as output
	PORTB |= _BV(PB4);							// enable MISO pull up
	SPCR =  _BV(SPE) | _BV(MSTR) | _BV(SPR1);	// activate SPI and set device as MASTER and SPI - Fosc/64

	tmp = SPSR;									//clear SPI Interrupt Flag flags by reading SRSR and SPDR
	tmp = SPDR;
}

void Spi_TxRx(uint8_t* receiveArray, uint8_t numberbytes, ...) {
   uint8_t tempdata,i;

   PORTB &= ~_BV(PB2);   // set SS low (chip select)
   
   va_list arguments;
   va_start ( arguments , numberbytes );

   for(i=0; i<numberbytes; i++)
   {
      tempdata = va_arg(arguments, unsigned int);
      SPDR = tempdata;
      while(!(SPSR & (1<<SPIF)));
      receiveArray[i]=SPDR;
	  _delay_us(200);
   }

   va_end ( arguments );

   PORTB |= _BV(PB2);   // set SS high (chip select)
} 

void statuscheck (){
	LCDclr();
	LCDGotoXY(0,1);
	toneset++;
	toneset = toneset % TONESETS;
	Cmx823_Init();
}

void TimerInit(){
	TCCR1B =  _BV(CS12) | _BV(CS10);		// - Fosc/1024
	TCNT1 =  0;
	TIFR  =  _BV(OCF1A); 					// Clear OCF2/ Clear pending interrup
	TIMSK =  _BV(OCIE1A);					// Compare Match interrupt is enabled
	//TIMSK &=  ~_BV(OCIE1A);
	OCR1A  =  78;							// roughly 10 ms (7812.5 is 1 second)
}

void USART_Init(void)
{
	// Set baud rate
	UBRRH = (uint8_t)(USART_UBBR_VALUE>>8);
	UBRRL = (uint8_t)USART_UBBR_VALUE;
	//Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
	UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0);	
	//Enable Transmitter and Interrupt on receive complete
	UCSRB=(1<<TXEN);
}

static int put_char(char c, FILE *stream)
{
	loop_until_bit_is_set(UCSRA,UDRE); //wait for UDR to be clear
	UDR = c;
	return 0;
}