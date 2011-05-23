//	This is the standard GCS output file for ArduPilot
//ArduPilotMega 2.2 (JLN version) fully based on the ArduPilotMega v2.1
////////////////////////// JLN ARDUSTATION PROTOCOL ///////////////////////////////
// Updated on 05-02-11: JLN added embedded Ardustation Protocol for the Ardustation v1.2
/*
Message Prefixes
!!!		Position    		Low rate telemetry
+++		Attitude    		High rate telemetry
###		Mode        		Change in control mode
%%%		Waypoint    		Current and previous waypoints
XXX		Alert       		Text alert  - NOTE: Alert message generation is not localized to a function
PPP		IMU Performance		Sent every 20 seconds for performance monitoring

Message Suffix
***    All messages use this suffix
*/

// Units
// -----
//
// Unless indicated otherwise, numeric quantities use the following units:
//
// Measurement | Unit
// ------------+-------------------------------------
// angle       | degrees
// distance    | metres
// speed       | metres per second
// servo angle | microseconds
// voltage     | volts
// times       | seconds
// throttle    | percent

#if ADS_PROTOCOL == ADS_PROTOCOL_JLN

void print_attitude2ADS(void)
{
  	Serial3.println("");
 	Serial3.print("+++ASP:");
	Serial3.print(airspeed/100,DEC);
	Serial3.print(",TTH:");
	Serial3.print(g.channel_throttle.servo_out,DEC);
	Serial3.print (",RLL:");
	Serial3.print(dcm.roll_sensor/100,DEC);
	Serial3.print (",PCH:");
	Serial3.print(dcm.pitch_sensor/100,DEC);
	Serial3.println(",***");
 }

void print_position2ADS(void)
{       if (home_is_set) ready2nav = 1;

	Serial3.println("");
        Serial3.print("!!!LAT:");
	Serial3.print(current_loc.lat,DEC);
	Serial3.print(",LON:");
	Serial3.print(current_loc.lng,DEC); //wp_current_lat
	Serial3.print(",CRS:");
	Serial3.print(dcm.yaw_sensor/100,DEC);
	Serial3.print(",SPD:");
	Serial3.print(g_gps->ground_speed/100,DEC);		
	Serial3.print(",CRT:");
	Serial3.print(climb_rate,DEC);
	Serial3.print(",ALT:");
	Serial3.print(current_loc.alt/100,DEC);  // altitude AGL
	Serial3.print(",ALH:");
	Serial3.print((target_altitude/100),DEC); // altitude AGL

	Serial3.print(",BER:");
	Serial3.print(target_bearing/100,DEC);
	Serial3.print(",WPN:");
	Serial3.print(g.waypoint_index,DEC);//Actually is the waypoint.
	Serial3.print(",DST:");
	Serial3.print(wp_distance,DEC);
	Serial3.print(",BTV:");
	Serial3.print(battery_voltage,DEC);
	Serial3.print(",MOD:");
	Serial3.print(control_mode,DEC);
	Serial3.print(",FIX:");
	Serial3.print(ready2nav);		 // 1 = ready2nav
	Serial3.println(",***");
}

#endif
