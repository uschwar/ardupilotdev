#define GCS_PORT	    3
#define GCS_PROTOCOL	    GCS_PROTOCOL_MAVLINK   // QGroundControl protocol
//#define GCS_PROTOCOL	    GCS_PROTOCOL_NONE      // No GCS protocol to save memory
#define MAV_SYSTEM_ID	    1

// Add a ground start delay in seconds
//#define GROUND_START_DELAY  1

#define AIRSPEED_SENSOR	    DISABLED

//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
#define SERIAL0_BAUD        115200
#define SERIAL3_BAUD        115200

//////////////////////////////////////////////////////////////////////////////
// GPS_PROTOCOL 
#define GPS_PROTOCOL        GPS_PROTOCOL_AUTO

#define MAGNETOMETER	    ENABLED
#define MAG_ORIENTATION	    AP_COMPASS_COMPONENTS_UP_PINS_FORWARD
#define PARAM_DECLINATION   0.18  // Paris

//#define RC_5_FUNCT    RC_5_FUNCT_AILERON
//#define RC_6_FUNCT	RC_6_FUNCT_AILERON

//////////////////////////////////////////////////////////////////////////////
// THROTTLE_OUT                             DEBUG
//
// When debugging, it can be useful to disable the throttle output.  Set
// this option to DISABLED to disable throttle output signals.
//
// The default is to not disable throttle output.
//
#define THROTTLE_OUT        ENABLED

#define HIL_MODE            HIL_MODE_DISABLED    //  Configure for standard flight.
#define HIL_PORT	    0
#define HIL_PROTOCOL	    HIL_PROTOCOL_MAVLINK 

