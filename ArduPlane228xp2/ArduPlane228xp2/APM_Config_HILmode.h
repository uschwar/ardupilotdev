// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// THIS IS A SAMPLE CONFIGURATION FILE FOR DOING HARDWARE IN THE LOOP TESTING USING THE ORIGINAL X-PLANE INTERFACE
// IF YOU WANTED TO USE THIS YOU WOULD COPY THE CONTENTS INTO YOUR APM_Config.h FILE!


#define FLIGHT_MODE_CHANNEL	8

#define X_PLANE  ENABLED

//#define HIL_PROTOCOL        HIL_PROTOCOL_XPLANE

//#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PROTOCOL        GCS_PROTOCOL_NONE
#define GCS_PORT            3

#define HIL_PROTOCOL        HIL_PROTOCOL_MAVLINK

//#define HIL_MODE            HIL_MODE_DISABLED
#define HIL_MODE            HIL_MODE_ATTITUDE
#define HIL_PORT            0

#define AIRSPEED_CRUISE     10     
#define STANDARD_SPEED	    10  
#define THROTTLE_FAILSAFE   DISABLED
#define AIRSPEED_SENSOR	    DISABLED
#define MAGNETOMETER	    DISABLED
#define LOGGING_ENABLED	    DISABLED

#define THROTTLE_CRUISE	    45
#define THROTTLE_MAX	    100
#define THROTTLE_MIN	    0

#define WP_RADIUS_DEFAULT     30
#define LOITER_RADIUS_DEFAULT 30
#define ALT_HOLD_HOME         150

