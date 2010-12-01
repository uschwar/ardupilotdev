#if GCS_OUTPUT == 1

//	This is the standard GCS output file for ArduPilot

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

void print_attitude(void)
{
 //	        Serial.println("+++ASP:0,THH:22,RLL:0:PCH:0,***");  // test line OK with the happykillmore GCS

	    Serial.print("+++ASP:");
	    Serial.print((int)(GPS.ground_speed/100));  
		Serial.print(",THH:");
		Serial.print((int)((((float)ch3-1100)/800)*100));
		Serial.print(",RLL:");
		Serial.print((int)ToDeg(roll));
		Serial.print(",PCH:");
		Serial.print((int)ToDeg(pitch));
		Serial.print(",YAW:");
		Serial.print((int)ToDeg(yaw));
		Serial.print(",CRS:");
		Serial.print((int)ToDeg(yaw));
//		Serial.print((int)ToDeg(APM_Compass.Heading));

	        Serial.println(",***");

}
/*
void print_position(void)
{
	Serial.print("!!!");
	Serial.print("LAT:");
	Serial.print((int(GPS.ground_speed/100.0));
	Serial.print(",LON:");
	Serial.print(GPS.longitude/10,DEC); //wp_current_lat
	Serial.print(",SPD:");
	Serial.print((int)(GPS.ground_speed/100));		
	Serial.print(",ALT:");
	Serial.print(GPS.altitude/100.0,DEC);
	Serial.print(",CRS:");
	Serial.print((int)ToDeg(yaw));
	Serial.print(",TOW:");
	Serial.print(GPS.time);
	Serial.println(",***");
}
*/
long convert_to_dec(float x)
{
  return x*10000000;
}
#endif
