 
 // -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If you wish to change any of the setup parameters from
// their default values, place the appropriate #define statements here.

// For example if you wanted the Port 3 baud rate to be 38400 you would add a statement like the one below (uncommented)
//#define SERIAL3_BAUD        38400
//#define GCS_PROTOCOL        GCS_PROTOCOL_NONE

#define CONFIG_APM_HARDWARE APM_HARDWARE_APM1

#define CLI_ENABLED         ENABLED
#define CLI_SLIDER_ENABLED  DISABLED
#define CLOSED_LOOP_NAV     DISABLED
#define AUTO_WP_RADIUS      ENABLED

#include "APM_Config_HILmode.h"
//#include "APM_Config_Flight.h"

// The following are the recommended settings for Xplane simulation. Remove the leading "/* and trailing "*/" to enable:

/*
#define HIL_PROTOCOL        HIL_PROTOCOL_MAVLINK
#define HIL_MODE            HIL_MODE_ATTITUDE
#define HIL_PORT            0
#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PORT            3
*/


