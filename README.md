# ATmega328P.Experiments
Various programs written in embedded C for the ATmega328p microcontroller.

Program Descriptions:

DAC and SPI:
	This program ultilizes a dac (via SPI, model Microchip MCP4921), a potentiometer, a LED, and a 16x2 char LCD.
	The program polls the voltage of the potentiometer, and then requests the DAC output a
	linearly scaled voltage appropriate for controling an LED that operates between 1.5V-5V.
	The ADC alternates sampling between the DAC output and the potentiometer output, which is then displayed on the 16x2 LCD

PWM:
  	This program utilizes a potentiometer, a linear voltage photocell, a piezo buzzer, a LED, and a 16x2 LCD.
	The ADC alternates sampling the photocell and potentiometer, which are used to control the LED and piezo.
	The photocell reading is used to scale a PWM duty cycle according to a linear approiximated dimming curve, such that
		the brightness of the LED appears to linearly scale with the measured brightness of the photocell.
	The potentiometer reading is used to scale the wave length of a PWM output to a NPN Transistor to control a piezo buzzer to output between
	the two notes C6 (1046Hz) and C7 (2093Hz).
	The LCD will display the voltage readings of the two ADC inputs.
 
 
