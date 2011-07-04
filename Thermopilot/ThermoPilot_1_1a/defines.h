// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

// for internal sim
// ----------------
#define TURNRATE 90 		// (degrees) how fast we turn per second in degrees at full bank
#define CLIMBRATE_UP 1000 	// (meters * 100) how fast we climb in simulator at 90° 
#define CLIMBRATE_DOWN 3000 	// (meters * 100) how fast we climb in simulator at 90° 

// GPS baud rates
// --------------
#define NO_GPS		38400
#define NMEA_GPS	38400
#define EM406_GPS	57600
#define UBLOX_GPS	38400
#define ARDU_IMU	38400
#define MTK_GPS		38400
#define SIM_GPS		38400
#define GPS_EMUL	9600

// Radio channels
// Note channels are from 0!  
#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THROTTLE 2
#define CH_RUDDER 3

#define WP_START_BYTE 0x18		// where in memory home WP is stored + all other WP
#define WP_SIZE 10
#define IR_MAX_FIX .88

// PID enumeration
// ---------------
#define CASE_SERVO_ROLL 0
#define CASE_SERVO_PITCH 1
#define CASE_SERVO_RUDDER 2
#define CASE_NAV_ROLL 3
#define CASE_NAV_PITCH_ASP 4
#define CASE_NAV_PITCH_ALT 5
#define CASE_TE_THROTTLE 6
#define CASE_ALT_THROTTLE 7

// Feedforward cases
// ----------------
#define CASE_PITCH_COMP 0
#define CASE_RUDDER_MIX 1
#define CASE_P_TO_T 2

// Auto Pilot modes
// ----------------
#define MANUAL 0
#define CIRCLE 1			 // When flying sans GPS, and we loose the radio, just circle
#define STABILIZE 2

#define FLY_BY_WIRE_A 5		// Fly By Wire A has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical = manual throttle
#define FLY_BY_WIRE_B 6		// Fly By Wire B has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical => desired airspeed
				// Fly By Wire B = Fly By Wire A if you have AIRSPEED_SENSOR 0
#define AUTO      10
#define RTL       11
#define LOITER    12
#define TAKEOFF   13
#define LAND      14
#define HEADALT   15              // Lock the current heading and altitude
#define SARSEC    16              // Run a SAR type Sector Pattern

// Waypoint Modes
// ----------------
#define ABS_WP 0
#define REL_WP 1

// Events
// ------
#define EVENT_WILL_REACH_WAYPOINT 1
#define EVENT_SET_NEW_WAYPOINT_INDEX 2
#define EVENT_LOADED_WAYPOINT 3
#define EVENT_LOOP 4

//GPS_fix
#define VALID_GPS 0x00
#define BAD_GPS 0x01
#define FAILED_GPS 0x03

#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))/0.000294117

#define AIRSPEED_PIN 3		// Need to correct value
#define BATTERY_PIN 5		// Need to correct value

#define BLUE_LED_PIN 12			//36 = B,	37 = A,	35 = C

#define HOLD_ALT_ABOVE_HOME 8 // bitmask value

//  GCS Message ID's
#define MSG_ACKNOWLEDGE 0x00
#define MSG_HEARTBEAT 0x01
#define MSG_ATTITUDE 0x02
#define MSG_LOCATION 0x03
#define MSG_PRESSURE 0x04
#define MSG_STATUS_TEXT 0x05
#define MSG_PERF_REPORT 0x06
#define MSG_COMMAND 0x22
#define MSG_VALUE 0x32
#define MSG_PID 0x42
#define MSG_TRIMS 0x50
#define MSG_MINS 0x51
#define MSG_MAXS 0x52


//EEPRROM 
#define EE_CONFIG 0x00
#define EE_AP_OFFSET 0x01
#define EE_ROLL_TRIM 0x03
#define EE_PITCH_TRIM 0x04
#define EE_MAX_ALT 0x05
#define EE_MAX_SPEED 0x07
#define EE_WP_TOTAL 0x09
#define EE_WP_INDEX 0x0A
#define EE_WP_RADIUS 0x0B
#define EE_HOME_ALT 0x0C
#define EE_HOME_LAT 0x0E
#define EE_HOME_LNG 0x12
#define EE_ALT_HOLD_HOME 0x16

#define EE_CH3_TT 0x384

#define EE_CH1_TRIM 0x386
#define EE_CH2_TRIM 0x388
#define EE_CH3_TRIM 0x388
#define EE_CH4_TRIM 0x38C  // fix

#define EE_CH1_MIN 0x38E
#define EE_CH2_MIN 0x390
#define EE_CH3_MIN 0x392
#define EE_CH4_MIN 0x394  // fix

#define EE_CH1_MAX 0x396
#define EE_CH2_MAX 0x398
#define EE_CH3_MAX 0x39A
#define EE_CH4_MAX 0x39C  // fix

#define EE_ELEVON1_TRIM 0x39E
#define EE_ELEVON2_TRIM 0x3A0
#define EE_IR_MAX 0x3A2
