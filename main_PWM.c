/*
	This program utilizes a potentiometer, a linear voltage photocell, a piezo buzzer, a LED, and a 16x2 LCD.
	The ADC alternates sampling the photocell and potentiometer, which are used to control the LED and piezo.
	The photocell reading is used to scale a PWM duty cycle according to a linear approiximated dimming curve, such that
		the brightness of the LED appears to linearly scale with the measured brightness of the photocell.
	The potentiometer reading is used to scale the wave length of a PWM output to a NPN Transistor to control a piezo buzzer to output between
	the two notes C6 (1046Hz) and C7 (2093Hz).
	The LCD will display the voltage readings of the two ADC inputs.
*/

#include <avr/io.h>
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <util/delay.h>
#include <avr/interrupt.h>

static volatile unsigned int adcResultPotentiometer;
static volatile unsigned int adcResultPhotocell;
static volatile char ADCEventFlagPhotocell;
static volatile char ADCEventFlagPotentiometer;
static volatile int piezowavelength;
enum PollingSource {photocell, potentiometer};
static enum PollingSource pollsource;

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
	LcdCommandWrite(0x80); // Move cursor to first Row
	const char numericCodes[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
	num*=100;
	int number = num;
	int i0, i1, i2;
	i2 = number/100; //highest decimal digit
	i1 = (number/10)%10; //middle decimal digit
	i0 = number%10;  //lowest decimal digit

	LcdDataWrite('P');
	LcdDataWrite('h');
	LcdDataWrite('o');
	LcdDataWrite('t');
	LcdDataWrite('o');
	LcdDataWrite('c');
	LcdDataWrite('e');
	LcdDataWrite('l');
	LcdDataWrite('l');
	LcdDataWrite(' ');

	LcdDataWrite(numericCodes[i2]);
	LcdDataWrite(46); // '.'
	LcdDataWrite(numericCodes[i1]);
	LcdDataWrite(numericCodes[i0]);

	LcdDataWrite('V');

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

	LcdDataWrite('P');
	LcdDataWrite('O');
	LcdDataWrite('T');
	LcdDataWrite('.');
	LcdDataWrite(' ');
	LcdDataWrite(' ');
	LcdDataWrite(' ');
	LcdDataWrite(' ');
	LcdDataWrite(' ');
	LcdDataWrite(' ');

	LcdDataWrite(numericCodes[i2]);
	LcdDataWrite(46); // '.'
	LcdDataWrite(numericCodes[i1]);
	LcdDataWrite(numericCodes[i0]);

	LcdDataWrite('V');

}

ISR(ADC_vect){
	if (!(ADCSRA&ADIF))
		return;
	if (pollsource == potentiometer){
		adcResultPotentiometer=ADCH;
		// function linearly scaling petentiometer voltage (.76v to 5v) to freq to wavelength
		piezowavelength = (1000000/(((2093-1046)/(255-38))*(adcResultPotentiometer-38)+1046));
		OCR1B = piezowavelength;
		OCR1A = piezowavelength*2;
		ADMUX  = 0b01100101; //set mux to adc5
		pollsource = photocell; // next poll will be from photocell
		ADCEventFlagPotentiometer=1;
		
	}
	else { // adc result is from photocell
		adcResultPhotocell=ADCH;
		ADMUX  = 0b01100100; //set mux to adc4
		pollsource = potentiometer; //next poll will be from potentiometer
		ADCEventFlagPhotocell=1;
	}
	ADCSRA |= 1<<ADSC; // we need to set the conversion bit high again
	
}

unsigned char lerpLED(unsigned char x,unsigned char dMax,unsigned char dMin,unsigned char rMax,unsigned char rMin){
	// function to inverse scale x linearly from domain to range
	unsigned char result;
	result = rMin + ( ( (rMax-rMin) * (x-dMin) )/(dMax - dMin) );
	return result;
}

unsigned char scaleForLEDPWM(unsigned char x){
	// using linear interpolation to estimate a dimming curve
	unsigned char result;
	x = 255-x; // duty cycle inverse of voltage read by adc (brighter = brighter)
	unsigned char inpoint0,inpoint1,inpoint2,inpoint3,inpoint4,inpoint5;
	// input points for lerp
	// represents y values of the curve
	inpoint0 = 0;
	inpoint1 = 21;
	inpoint2 = 51;
	inpoint3 = 100;
	inpoint4 = 170;
	inpoint5 = 255;
	unsigned char outpoint0,outpoint1,outpoint2,outpoint3,outpoint4,outpoint5;
	// output points for lerp estimation curve
	// represent x values of the curve
	// approximate an exponential curve approaching an asymptote
	// these points correspond to an approximately linear PERCIEVED brightness
	outpoint0 = 1;
	outpoint1 = 140;
	outpoint2 = 200;
	outpoint3 = 200;
	outpoint4 = 235;
	outpoint5 = 254;
	if (x<inpoint1)
	result = lerpLED(x,inpoint0,inpoint1,outpoint0,outpoint1);
	else if (x<inpoint2)
	result = lerpLED(x,inpoint1,inpoint2,outpoint1,outpoint2);
	else if (x<inpoint3)
	result = lerpLED(x,inpoint2,inpoint3,outpoint2,outpoint3);
	else if (x<inpoint4)
	result = lerpLED(x,inpoint3,inpoint4,outpoint3,outpoint4);
	else
	result = lerpLED(x,inpoint4,inpoint5,outpoint4,outpoint5);
	return result;

}



int main(void){
	/* Initialize Registers for LCD */
	DDRD |=1<<DDD0;
	DDRD |=1<<DDD1;
	DDRC |=1<<DDC3;
	DDRC |=1<<DDC2;
	DDRC |=1<<DDC1;
	DDRC |=1<<DDC0;

	/* Initialize LCD Vars*/

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
	PRR |= 0<<PRADC; //disable power reduction adc bit
	ADMUX  = 0b01100101; // use 5v reference voltage, left adjust, and set mux to adc5
	ADCSRA = 0b10001111; // enable the adc, enable interrupts, and start a conversion, set prescalar to 16 
	ADCSRB = 0b00000000; // using no options from this reg

	/* initalize pwm for led*/
	DDRD |= 1<<DDD6; //enable OC0A output pin
	TCCR0A = 0b11000011; //enable OC0A set at bottom, clear at top. Set fast PWM mode
	TCCR0B = 0b00000001; // prescalar set to 8
	OCR0A = 1; // init with 100% duty cycle

	/* initialize pwm for piezo buzzer*/
	DDRB |= 1<<DDB2; // enable PB2/OC1B for output
	TCCR1A = 0b00100011; // enable OC1B set at bottom, clear at comp match. set fast PWM mode
	TCCR1B = 0b00011010;
	OCR1B = 1046; // half period (clk goes low when timer = OCR1B)
	OCR1A = 1046*2; // period (clock counts to OCR1A)


	/* Initialize Vars*/
	float voltageAmtPhotocell;
	float voltageAmtPotentiometer;
	pollsource = potentiometer;
	//
	sei();
	ADCSRA |= 1<<ADSC; // start adc conversion
	while (1){	
		if (ADCEventFlagPhotocell){
			ADCEventFlagPhotocell=0;
			OCR0A = scaleForLEDPWM(adcResultPhotocell); // Alter duty% on PWM to be the inverse of the brightness from photoresistor
			voltageAmtPhotocell = adcResultPhotocell; //convert adc measurement to voltage measurement
			voltageAmtPhotocell = voltageAmtPhotocell/51;
			LcdWriteFloatLine1(voltageAmtPhotocell);
		}
		if (ADCEventFlagPotentiometer){
			ADCEventFlagPotentiometer=0;
			voltageAmtPotentiometer = adcResultPotentiometer;
			voltageAmtPotentiometer = voltageAmtPotentiometer/51;
			LcdWriteFloatLine2(voltageAmtPotentiometer);
		}
	}
}