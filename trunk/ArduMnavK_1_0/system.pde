/*****************************************************************************
The init_ardupilot function processes everything we need for an in-air restart
	We will determine later if we are actually on the ground and process a 
	ground start in that case.
	
Also in this function we will dump the log if applicable based on the slide switch
*****************************************************************************/
void init_ardumnav()
{
	digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
        Serial.begin(38400);
	Serial.println();
 	Serial.println("ArduMnav Kalman v1.0 JLN");
	
 	
 	// ATMEGA
	// PORTD
	// p0				// PD0 - RXD  		- Serial RX 	
	// p1				// PD1 - TXD  		- Serial TX 
	pinMode(2,INPUT);	// PD2 - INT0 		- Rudder in							- INPUT Rudder/Aileron
	pinMode(3,INPUT);	// PD3 - INT1 		- Elevator in 						- INPUT Elevator
	pinMode(4,INPUT);	// PD4 - XCK/T0 	- MUX pin							- Connected to Pin 2 on ATtiny
	pinMode(5,INPUT);	// PD5 - T0			- Mode pin							- Connected to Pin 6 on ATtiny   - Select on MUX
	pinMode(6,OUTPUT);	// PD6 - T1			- Ground start signaling Pin	
	pinMode(7,OUTPUT);	// PD7 - AIN0		- GPS Mux pin 
	// PORTB
	pinMode(8, OUTPUT); // PB0 - AIN1		- Servo throttle					- OUTPUT THROTTLE
	pinMode(9, OUTPUT);	// PB1 - OC1A		- Elevator PWM out					- Elevator PWM out
	pinMode(10,OUTPUT);	// PB2 - OC1B		- Rudder PWM out					- Aileron PWM out
	pinMode(11,INPUT); 	// PB3 - MOSI/OC2	-  
	pinMode(12,OUTPUT); // PB4 - MISO		- Blue LED pin  - GPS Lock			- GPS Lock
	pinMode(13,INPUT); 	// PB5 - SCK		- Yellow LED pin   					- INPUT Throttle

	// PORTC - Analog ports
	// PC0 - Thermopile - x
	// PC1 - Thermopile - y 
	// PC2 - Thermopile - z
	// PC3 - Airspeed
	// PC4 - CH4 OUT - Rudder output
	// PC5 - Battery

	// set Analog out 4 to output
	DDRC |= B00010000;
	
	digitalWrite(6,HIGH);

	Serial.print("freeRAM: ");
	Serial.println(freeRAM(),DEC);

	kalmanReset();

}

byte startup_check(void){
}


void setCommandMux(void)
{
	#if SHIELD_VERSION < 1
		digitalWrite(7, HIGH); //Remove Before Fly Pull Up resistor
    #else
		digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
	#endif
}



/* This function gets the current value of the heap and stack pointers.
* The stack pointer starts at the top of RAM and grows downwards. The heap pointer
* starts just above the static variables etc. and grows upwards. SP should always
* be larger than HP or you'll be in big trouble! The smaller the gap, the more
* careful you need to be. Julian Gall 6-Feb-2009.
*/
unsigned long freeRAM() {
	uint8_t * heapptr, * stackptr;
	stackptr = (uint8_t *)malloc(4); // use stackptr temporarily
	heapptr = stackptr; // save value of heap pointer
	free(stackptr); // free up the memory again (sets stackptr to 0)
	stackptr = (uint8_t *)(SP); // save value of stack pointer
	return stackptr - heapptr;
}



