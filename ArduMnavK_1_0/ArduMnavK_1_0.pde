// ArduMnav 1.0 - ArduPilot datalink interface with Crossbow mNav IMU
// october 20, 2010 by Jean-Louis Naudin
// This mNav interface use a Kalman filtering algorithm for the IMU sensors
//
// Updated on 21-10-10 JLN - 
/* 
     This program is free software: you can redistribute it and/or modify 
     it under the terms of the GNU General Public License as published by 
     the Free Software Foundation, either version 3 of the License, or 
     (at your option) any later version. 
 
      This program is distributed in the hope that it will be useful, 
      but WITHOUT ANY WARRANTY; without even the implied warranty of 
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
      GNU General Public License for more details. 
 
      You should have received a copy of the GNU General Public License 
      along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
// ************************************************************************* 
// More info at: http://diydrones.com/profile/JeanLouisNaudin

#include <avr/io.h>
#include <avr/eeprom.h>
#include <math.h>
#include "defines.h"

//To use the header file in your local folder, use quotes:
#include "AR_Config.h"

// GENERAL VARIABLE DECLARATIONS
// --------------------------------------------
uint16_t timer_ovf_a	= 0;
uint16_t timer_ovf_b	= 0;
uint16_t timer_ovf	= 0;

union long_union {
	int32_t dword;
	uint8_t	byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t	byte[2];
} intUnion;

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi


// System Timers
// --------------
unsigned long fast_loopTimer		= 0;		// Time in miliseconds of main control loop
unsigned long medium_loopTimer		= 0;		// Time in miliseconds of navigation control loop
byte medium_loopCounter			= 0;		// Counters for branching from main control loop to slower loops
byte slow_loopCounter			= 0;		// 
unsigned long deltaMiliSeconds 		= 0;		// Delta Time in miliseconds
unsigned long dTnav			= 0;		// Delta Time in milliseconds for navigation computations
int mainLoop_count 			= 0;
unsigned long elapsedTime		= 0;		// for doing custom events

// Mnav IMU
double ax		= 0;  // IMU Accel in m/s^2
double ay		= 0;
double az		= 0;
double p		= 0;  // IMU angular rate in rad/s
double q		= 0;
double r		= 0;
double hx		= 0;  // IMU magnetic field in gauss
double hy		= 0;
double hz		= 0;
double Ps		= 0;  // pressure in m    (alt)
double Pt		= 0;  // pressure in m/s  (speed)

int		angleN;			// nick (front/back) angle in degrees
int		angleR;			// roll (left/right) angle in degrees

int MNAV_messages_received = 0;
byte MNAV_payload_error_count = 0;
byte MNAV_checksum_error_count = 0;
boolean MNAV_ok = false;		// the MNAV is sending data correctly


// Basic Initialization
//---------------------
void setup() {
	init_ardumnav();
}

void loop()
{
	// We want this to execute at 50Hz if possible  (20 ms)
	// -------------------------------------------
	if (DIYmillis()-fast_loopTimer > 19) {
		deltaMiliSeconds 	= DIYmillis() - fast_loopTimer;
		fast_loopTimer		= DIYmillis();

		// Execute the fast loop
		// ---------------------
		//PORTD |= B10000000;
		fast_loop();
		//PORTD &= B01111111;
		
		// Execute the medium loop 
		// -----------------------
		//PORTD |= B01000000;
		medium_loop();
		//PORTD &= B10111111;

		mainLoop_event();
	}
}

void fast_loop()
{

}

void medium_loop()
{
	// This is the start of the medium (10 Hz) loop pieces (100 ms)
	// -----------------------------------------
	switch (medium_loopCounter){
	
		// This case Read Mnav data
		//-------------------------
		case 0:
                	read_Mnav();
                        readSensors();	
                        state_update();
                        accel2angle();
                        kalmanFilter(); 
			medium_loopCounter++;
			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:

			medium_loopCounter++;
			break;

		// unused
		//------------------------------
		case 2:                      
			medium_loopCounter++;
			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;            
			break;
			
		// This case controls the slow loop
		//---------------------------------
		case 4:
                	Kalman_result();
                        // Mnavdata_Out(); 
			medium_loopCounter=0;
			mediumLoop_event();
			slow_loop();
			break;	
	}
}

void slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces  (300ms)
	//----------------------------------------
	switch (slow_loopCounter){
		case 0:
			slow_loopCounter++;
			// nothing here
			break;
			
		case 1:
			slow_loopCounter++;

			break;
			
		case 2:
			slow_loopCounter = 0;
			// Reserved
			// Probably put uplink here
			break;
	}
}

