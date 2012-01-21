////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////  PID SETUP FOR THE Quad Rotor Observer of Jean-Louis Naudin //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Frame: Home made quadcopter 550 mm wide, plus configuration

Avionic setup:
 - Ardupilot Mega (Atmega 2560 @16 MHz), 32 Mips
 - Full 9DOF IMU,
 - Built-in 16 MB Data Logger,
 - Absolute pressure sensors (Bosh) for altitude,
 - triple axis magnetometer HMC5843,
 - 10Hz Mediatek GPS.
 - firmware Arducopter V2.1 (original version

 - Receiver: Turnigy 9X8C v2
 - Transmitter: Turnigy 9x
 - uBec 5V @ 5A Turnigy 

Hardware Setup:
- four brushless motors RC Timer BC-2836/11 (750 KV)
- two CW propellers  12 x 4.5
- two CCW propellers 12 x 4.5
- fours ESC 40A RC Timer speed controllers
- one Lipo battery 3S Turnigy Nano-Tech 2200 mAh

- Take Off Weight (TOW): 988 g without payload

- Flight time: about 10 min
*/

#define FRAME_ORIENTATION PLUS_FRAME
	/*
	PLUS_FRAME
	X_FRAME
	V_FRAME
	*/

#define MAGNETOMETER		ENABLED
#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_UP_PINS_FORWARD
#define PARAM_DECLINATION       -0.433  // Paris

#define WIND_COMP_STAB 0

//#define CONFIG_SONAR DISABLED

#define LOGGING_ENABLED		ENABLED

#define STABILIZE_ROLL_P 	2.1      // 4.6
#define STABILIZE_ROLL_I 	0.1      // 0.02
#define STABILIZE_ROLL_IMAX 	30	 // 40  	// degrees

#define STABILIZE_PITCH_P	2.1      // 4.6
#define STABILIZE_PITCH_I	0.1      // 0.02
#define STABILIZE_PITCH_IMAX	30	 // 40    	// degrees

#define STABILIZE_D 		.25

//////////////////////////////////////////////////////////////////////////////
// Acro Rate Control
//
#define ACRO_ROLL_P             0.151    // 0.155
#define ACRO_ROLL_I             0.020    // 0.0
#define ACRO_ROLL_IMAX	 	5.0	 // 15   	// degrees

#define ACRO_PITCH_P            0.151    // 0.155
#define ACRO_PITCH_I		0.020    // 0.0
#define ACRO_PITCH_IMAX   	5.0	 // 15  	// degrees

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//

#define RATE_ROLL_P             0.079   // 0.155
#define RATE_ROLL_I             0.020   // 0.0
#define RATE_ROLL_IMAX	 	5.0	// 15    	// degrees

#define RATE_PITCH_P            0.079   // 0.155
#define RATE_PITCH_I		0.020   // 0.0
#define RATE_PITCH_IMAX   	5.00	// 15    	// degrees

//////////////////////////////////////////////////////////////////////////////
// YAW Control
//
#define STABILIZE_YAW_P		7.00	// 7.00  	// increase for more aggressive Yaw Hold, decrease if it's bouncy
#define STABILIZE_YAW_I		0.01	// 0.01  	// set to .0001 to try and get over user's steady state error caused by poor balance
#define STABILIZE_YAW_IMAX	8.00	// 8	        // degrees * 100

#define RATE_YAW_P              0.13	// 0.13  	// used to control response in turning
#define RATE_YAW_I              0.00    // 0.0
#define RATE_YAW_IMAX           50.0    // 50

//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
//
#define LOITER_P		0.5		// 2.0
#define LOITER_I		0.001	        // 0.05
#define LOITER_IMAX		10.0		// 30        // degrees

#define NAV_P			2.20		// 2.20
#define NAV_I			0.15		// 0.15      // Lowerd from .25 - saw lots of overshoot.
#define NAV_IMAX		30.0		// 30        // degrees

#define WAYPOINT_SPEED_MAX	600		// 600       // for 6m/s error = 13mph

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//
#define THROTTLE_CRUISE	        348             // 350
		
#define THR_HOLD_P		0.396		// 0.4       // Altitude Hold
#define THR_HOLD_I		0.020		// 0.01      // with 4m error, 12.5s windup
#define THR_HOLD_IMAX	        300             // 300

// RATE control
#define THROTTLE_P		0.195           // 0.4
#define THROTTLE_I		0.015           // 0.0
#define THROTTLE_IMAX	        50             // 300

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
#define CROSSTRACK_GAIN		4               // 1

#define WP_RADIUS_DEFAULT	1

