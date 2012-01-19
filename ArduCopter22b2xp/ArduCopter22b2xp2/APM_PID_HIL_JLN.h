////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////  PID SETUP FOR THE Quad Rotor Observer of Jean-Louis Naudin //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
QRO_X a simulation for X-Plane or AeroSIM-RC by Jean-Louis Naudin - Dec 28, 2011
*/

#define FRAME_ORIENTATION PLUS_FRAME
	/*
	PLUS_FRAME
	X_FRAME
	V_FRAME
	*/

#define HIL_MODE		HIL_MODE_ATTITUDE
#define LOGGING_ENABLED		DISABLED
#define CONFIG_SONAR            DISABLED

#define WIND_COMP_STAB 0

#define X_PLANE                 ENABLED   // required to run in HIL mode with X-Plane and AeroSIM-RC

#define MAGNETOMETER		DISABLED

#define STABILIZE_ROLL_P 	4.6      // 4.6
#define STABILIZE_ROLL_I 	0.02     // 0.02
#define STABILIZE_ROLL_IMAX 	30	 // 40  	// degrees

#define STABILIZE_PITCH_P	4.6      // 4.6
#define STABILIZE_PITCH_I	0.02      // 0.02
#define STABILIZE_PITCH_IMAX	30	 // 40    	// degrees

//////////////////////////////////////////////////////////////////////////////
// Acro Rate Control
//
#define ACRO_ROLL_P             0.145    // 0.145
#define ACRO_ROLL_I             0.000    // 0.0
#define ACRO_ROLL_IMAX	 	15.0	 // 15   	// degrees

#define ACRO_PITCH_P            0.145    // 0.145
#define ACRO_PITCH_I		0.000    // 0.0
#define ACRO_PITCH_IMAX   	15.0	 // 15  	// degrees

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//

#define RATE_ROLL_P             0.145   // 0.145
#define RATE_ROLL_I             0.000   // 0.0
#define RATE_ROLL_IMAX	 	15.0	// 15    	// degrees

#define RATE_PITCH_P            0.145   // 0.145
#define RATE_PITCH_I		0.000   // 0.0
#define RATE_PITCH_IMAX   	15.00	// 15    	// degrees

//////////////////////////////////////////////////////////////////////////////
// YAW Control
//
#define STABILIZE_YAW_P		1.00	// 7.00  	// increase for more aggressive Yaw Hold, decrease if it's bouncy
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
#define LOITER_IMAX		30.0		// 30        // degrees

#define NAV_P			2.20		// 2.2
#define NAV_I			0.15		// 0.15      // Lowerd from .25 - saw lots of overshoot.
#define NAV_IMAX		30.0		// 30        // degrees

#define WAYPOINT_SPEED_MAX	600		// 600       // for 6m/s error = 13mph

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//
#define THROTTLE_CRUISE	        350             // 350
		
#define THR_HOLD_P		0.400		// 0.4       // Altitude Hold
#define THR_HOLD_I		0.010		// 0.01      // with 4m error, 12.5s windup
#define THR_HOLD_IMAX	        300             // 300

// RATE control
#define THROTTLE_P		0.500           // 0.5
#define THROTTLE_I		0.000           // 0.0
#define THROTTLE_IMAX	        300             // 300

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
#define CROSSTRACK_GAIN		4               // 1

#define WP_RADIUS_DEFAULT	1
