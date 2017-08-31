/*
	This program ultilizes a dac (via SPI, model Microchip MCP4921), a potentiometer, a LED, and a 16x2 char LCD.
	The program polls the voltage of the potentiometer, and then requests the DAC output a
	linearly scaled voltage appropriate for controling an LED that operates between 1.5V-5V.
	The ADC alternates sampling between the DAC output and the potentiometer output, which is then displayed on the 16x2 LCD
*/

#include <avr/io.h>
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <util/delay.h>
#include <avr/interrupt.h>

static volatile char ADCEventFlagPotentiometer;
static volatile unsigned int adcResultPotentiometer;

static volatile char ADCEventFlagDAC;
static volatile unsigned int adcResultDAC;

enum PollingSource {dac, potentiometer};
static enum PollingSource pollsource;

static volatile char SPIWriteAllowed;

void LcdCommandWrite_UpperNibble(unsigned char data){
	/* write upper nibble*/
	PORTC = (PORTC & 0xf0) | (data >> 4);
	PORTD &= ~(1<<PORTD1); // RS = 0
	PORTD |= 1<<PORTD0; // E = 1
	_delay_ms(1);
	PORTD &= ~(1<<PORTD0);// E = 0
	_delay_ms(1);
}

void LcdCommandWrite(unsigned char data){
	/*write upper nibble*/
	PORTC = (PORTC & 0xf0) | (data >> 4);
	PORTD &= ~(1<<PORTD1); // RS = 0
	PORTD |= 1<<PORTD0; // E = 1
	_delay_ms(1);
	PORTD &= ~(1<<PORTD0);// E = 0
	_delay_ms(1);

	/* write lower nibble */
	PORTC = (PORTC & 0xf0) | (data & 0x0f);
	PORTD &= ~(1<<PORTD1); // RS = 0
	PORTD |= 1<<PORTD0; // E = 1
	_delay_ms(1);
	PORTD &= ~(1<<PORTD0);// E = 0
	_delay_ms(1);
}

void LcdDataWrite(unsigned char data){
	/*write upper nibble*/
	PORTC = (PORTC & 0xf0) | (data >> 4);
	PORTD |= 1<<PORTD1; // RS = 1
	PORTD |= 1<<PORTD0; // E = 1
	_delay_ms(1);
	PORTD &= ~(1<<PORTD0);// E = 0
	_delay_ms(1);

	/* write lower nibble */
	PORTC = (PORTC & 0xf0) | (data & 0x0f);
	PORTD |= 1<<PORTD1; // RS = 1
	PORTD |= 1<<PORTD0; // E = 1
	_delay_ms(1);
	PORTD &= ~(1<<PORTD0);// E = 0
	_delay_ms(1);
}

void LcdWriteFloatLine1(float num){
	LcdCommandWrite(0x80); // Move cursor to first row
	const char numericCodes[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
	num*=100;
	int number = num;
	int i0, i1, i2;
	i2 = number/100; //highest decimal digit
	i1 = (number/10)%10; //middle decimal digit
	i0 = number%10;  //lowest decimal digit

	LcdDataWrite('P');
	LcdDataWrite('O');
	LcdDataWrite('T');
	LcdDataWrite(':');
	LcdDataWrite(' ');
	LcdDataWrite(' ');


	LcdDataWrite(numericCodes[i2]);
	LcdDataWrite(46); // '.'
	LcdDataWrite(numericCodes[i1]);
	LcdDataWrite(numericCodes[i0]);

	LcdDataWrite(' ');
	LcdDataWrite('V');
	LcdDataWrite('o');
	LcdDataWrite('l');
	LcdDataWrite('t');
	LcdDataWrite('s');

}

void LcdWriteFloatLine2(float num){
	LcdCommandWrite(0xc0); // Move cursor to second row
	const char numericCodes[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
	num*=100;
	int number = num;
	int i0, i1, i2;
	i2 = number/100; //highest decimal digit
	i1 = (number/10)%10; //middle decimal digit
	i0 = number%10;  //lowest decimal digit

	LcdDataWrite('D');
	LcdDataWrite('A');
	LcdDataWrite('C');
	LcdDataWrite(':');
	LcdDataWrite(' ');
	LcdDataWrite(' ');


	LcdDataWrite(numericCodes[i2]);
	LcdDataWrite(46); // '.'
	LcdDataWrite(numericCodes[i1]);
	LcdDataWrite(numericCodes[i0]);

	LcdDataWrite(' ');
	LcdDataWrite('V');
	LcdDataWrite('o');
	LcdDataWrite('l');
	LcdDataWrite('t');
	LcdDataWrite('s');

}

void SpiDataWrite(unsigned int data){
	unsigned volatile char upperByte = 0b00110000; // DAC A select, unbuffered VRef input, gain Vout = Vref*D/4096, output enabled
	unsigned volatile char lowerByte;
	upperByte |= (0x0f & (data>>8)); // shift data to the appropriate registers
	lowerByte = data;
	PORTB &= ~(1<<PORTB2); // pull CS low while writing data
	_delay_us(0.1);
	SPDR = upperByte;
	while(!(SPSR & (1<<SPIF)));
	SPDR = lowerByte;
	while(!(SPSR & (1<<SPIF)));

	_delay_us(0.1);
	PORTB |= 1<<PORTB2; // return CS to high after write
	_delay_us(0.1);
	PORTB &= ~(1<<PORTB0); ////////////////////////////////////////////
	_delay_us(0.1); // pull LDAC low for latches on DAC to update //
	PORTB |= 1<<PORTB0; ///////////////////////////////////////////
	_delay_ms(1);
}


ISR(ADC_vect){
	if ((ADCSRA&(1<<ADIF)))
	return;

	if (pollsource == potentiometer)
	{ // adc result is from potentiometer
		adcResultPotentiometer = ADCL>>6;
		adcResultPotentiometer |= ADCH<<2;
		ADMUX  = 0b01100101; //set mux to adc5
		pollsource = dac; // next poll will be from dac
		ADCEventFlagPotentiometer=1;
	}
	else { // adc result is from dac
		adcResultDAC = ADCL>>6;
		adcResultDAC |= ADCH<<2;
		ADMUX  = 0b01100100; //set mux to adc4
		pollsource = potentiometer;
		ADCEventFlagDAC=1;
	}
	ADCSRA |= 1<<ADSC; // we need to set the conversion bit high again
}

int main(void)
{
	/* Initialize Data Direction Registers for LCD */
	DDRD |=1<<DDD0;
	DDRD |=1<<DDD1;
	DDRC |=1<<DDC3;
	DDRC |=1<<DDC2;
	DDRC |=1<<DDC1;
	DDRC |=1<<DDC0;

	/* Initialize LCD */
	LcdCommandWrite_UpperNibble(0x30);
	_delay_ms(4.1);
	LcdCommandWrite_UpperNibble(0x30);
	_delay_us(100);
	LcdCommandWrite_UpperNibble(0x30);
	LcdCommandWrite_UpperNibble(0x20);
	LcdCommandWrite(0x28);
	// function set: 0x28 means, 4-bit interface, 2 lines, 5x8 font
	LcdCommandWrite(0x08);
	// display control: turn display off, cursor off, no blinking
	LcdCommandWrite(0x01);
	// clear display, set address counter to zero
	LcdCommandWrite(0x06); // entry mode set:
	LcdCommandWrite(0x0C); // display on
	_delay_ms(120);

	/* init adc */
	ADMUX  = 0b01100100; // use 5v reference voltage, left adjust, and set mux to adc4
	ADCSRA = 0b10001111; // enable the adc, enable interrupts, and start a conversion, set prescalar to 16
	ADCSRB = 0b00000000; // using no options from this reg

	pollsource=potentiometer;

    /* init spi */
	DDRB |= 1<<DDB5;
	DDRB |= 1<<DDB3;
	DDRB |= 1<<DDB2;
	DDRB |= 1<<DDB0;
	PORTB |= 1<<PORTB2; // set CS high
	PORTB |= 1<<PORTB0; // set LDAC high
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);

	float voltageAmtPotentiometer;
	float voltageAmtDac;
	float tmp;
	unsigned int spioutdata;
	sei();
	ADCSRA |= 1<<ADSC; // start adc conversion

    while (1) 
    {
		if (ADCEventFlagPotentiometer){
			tmp = adcResultPotentiometer;
			spioutdata = (((4096.0-1228.8)/(1023.0-157.08))*(tmp-157.08)+1228.8); // scale potentiometer reading to appropriately sized int
			SpiDataWrite(spioutdata); // start an spi write operation to the dac
			ADCEventFlagPotentiometer=0;
			voltageAmtPotentiometer = adcResultPotentiometer;
			voltageAmtPotentiometer = voltageAmtPotentiometer/204;
			LcdWriteFloatLine1(voltageAmtPotentiometer);
			
		}
		if (ADCEventFlagDAC){
			ADCEventFlagDAC=0;
			voltageAmtDac = adcResultDAC;
			voltageAmtDac = voltageAmtDac/204;
			LcdWriteFloatLine2(voltageAmtDac);
		}
    }
}