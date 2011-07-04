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

/*
void acknowledge(byte id, byte check1, byte check2) {}
void send_message(byte id) {}
void send_message(byte id, long param) {}
void send_message(byte severity, const char *str) {}
*/

#if GCS_PROTOCOL == 6

void acknowledge(byte id, byte check1, byte check2)
{
}

void send_message(byte id) {
	send_message(id,0l);
}

void send_message(byte id, long param) {
	switch(id) {
		case MSG_ATTITUDE:								// ** Attitude message
                     // print_attitude();
                        output_gpsemulator();
			break;
		case MSG_LOCATION:								// ** Location/GPS message
                        //output_gpsemulator();
			print_position();
			break;
		case MSG_HEARTBEAT:								// ** Location/GPS message
			print_control_mode();
			break;
		case MSG_PERF_REPORT:
			printPerfData();
	}	
}
void send_message(byte severity, const char *str)
{

}

void print_current_waypoints(){
}

void print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			Serial.println("###MANUAL\t0***");
			break;
		case STABILIZE:
			Serial.println("###STABILIZE\t1***");
			break;
		case CIRCLE:
			Serial.println("###CIRCLE\t1***");
			break;
		case FLY_BY_WIRE_A:
			Serial.println("###FBW A\t2***");
			break;
		case FLY_BY_WIRE_B:
			Serial.println("###FBW B\t2***");
			break;
		case AUTO:
			Serial.println("###AUTO\t6***");
			break;
		case RTL:
			Serial.println("###RTL\t7***");
			break;
		case LOITER:
			Serial.println("###LOITER\t8***");
			break;
		case TAKEOFF:
			Serial.println("##TAKEOFF\t9");
			break;
		case LAND:
			Serial.println("##LAND\t10");
			break;
	}
}

void print_position(void)
{	Serial.print("!!!");
      	Serial.print("LAT:");
	Serial.print(current_loc.lat/10,DEC); //wp_current_lat
	Serial.print(",LON:");
	Serial.print(current_loc.lng/10,DEC); //wp_current_lon
	Serial.print(",ALT:");
	Serial.print(current_loc.alt/100,DEC);
	Serial.print(",ALH:");
	Serial.print(next_WP.alt/100,DEC);
	Serial.print(",CRS:");
	Serial.print(ground_course/100,DEC);
	Serial.print(",BER:");
	Serial.print(target_bearing/100,DEC);
	Serial.print(",XTR:");
	Serial.print((int)crosstrack_error,DEC);
	Serial.print(",WPN:");
	Serial.print(wp_index,DEC);//Actually is the waypoint.
	Serial.print(",DST:");
	Serial.print(wp_distance,DEC);
	Serial.print(",FIX:");
	Serial.print(GPS_fix,DEC);
	Serial.println(",***");
  /*
	Serial.print("!!!");
	Serial.print("LAT:");
	Serial.print(current_loc.lat/10,DEC);
	Serial.print(",LON:");
	Serial.print(current_loc.lng/10,DEC); //wp_current_lat
	Serial.print(",SPD:");
	Serial.print(ground_speed/100,DEC);		
	Serial.print(",CRT:");
	Serial.print(climb_rate,DEC);
	Serial.print(",ALT:");
	Serial.print(current_loc.alt/100,DEC);
	Serial.print(",ALH:");
	Serial.print(next_WP.alt/100,DEC);
	Serial.print(",CRS:");
	Serial.print(ground_course/100,DEC);
	Serial.print(",BER:");
	Serial.print(target_bearing/100,DEC);
	Serial.print(",WPN:");
	Serial.print(wp_index,DEC);//Actually is the waypoint.
	Serial.print(",DST:");
	Serial.print(wp_distance,DEC);
	Serial.print(",BTV:");
	Serial.print(battery_voltage,DEC);
	Serial.print(",RSP:");
	Serial.print(servo_out[CH_ROLL]/100,DEC);
	Serial.print(",TOW:");
	Serial.print(iTOW);
	Serial.println(",***");
*/
}


void printPerfData(void)
{

}

void print_attitude(void)
{
	Serial.print("+++");
	Serial.print("ASP:");
	Serial.print(airspeed,DEC);
	Serial.print(",THH:");
	Serial.print(servo_out[CH_THROTTLE],DEC);
	Serial.print (",RLL:");
	Serial.print(roll_sensor/100,DEC);
	Serial.print (",PCH:");
	Serial.print(pitch_sensor/100,DEC);
	#if GPS_PROTOCOL == 3
		Serial.print(",CRS:");
		Serial.print(ground_course/100,DEC);
		Serial.print(",IMU:");
		Serial.print(imu_health,DEC);
	#endif
	Serial.print(", in:");
	Serial.print(radio_in[CH_THROTTLE]);
	Serial.println(",***");
}

void output_gpsemulator(void)
{
    Serial.print("!!!");
/*
    	Serial.print("LAT:");
	Serial.print(current_loc.lat/10,DEC); //wp_current_lat
	Serial.print(",LON:");
	Serial.print(current_loc.lng/10,DEC); //wp_current_lon
	Serial.print(",ALT:");
	Serial.print(current_loc.alt/100,DEC);

	Serial.print(",FIX:");
	Serial.print(GPS_fix,DEC);
*/
    Serial.print (",STT:");
    Serial.print((int)readSwitch());
    Serial.print (",WPN:");
    Serial.print((int)wp_index,DEC);  //Actually is the waypoint.
    Serial.print (",DST:");
    Serial.print((int)wp_distance,DEC);
    Serial.print (",RER:");
    Serial.print((int)(nav_roll/100),DEC);
  //  Serial.print((int)(nav_roll - roll_sensor)/100,DEC);
    Serial.print (",PSET:");
    Serial.print((int)(nav_pitch/100),DEC);
    Serial.print(",THH:");
    Serial.print((int)servo_out[CH_THROTTLE],DEC);
    Serial.println(",***");
}

#endif
