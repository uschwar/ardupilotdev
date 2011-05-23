//ArduPilotMega 2.2 (JLN version) fully based on the ArduPilotMega v2.1
////////////////////////// REMBIZI GPS EMULATOR PROTOCOL ///////////////////////////////
// Updated on 05-10-11: JLN added GPS emulator protocol for nav simulation with the Rembizi GPSemulator v1.1.45

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

#if ADS_PROTOCOL == ADS_PROTOCOL_EMU

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
			print_position2EMU();
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

void print_current_waypoints()
{
}

void print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			Serial3.println("###MANUAL\t0***");
			break;
		case STABILIZE:
			Serial3.println("###STABILIZE\t1***");
			break;
		case CIRCLE:
			Serial3.println("###CIRCLE\t1***");
			break;
		case FLY_BY_WIRE_A:
			Serial3.println("###FBW A\t2***");
			break;
		case FLY_BY_WIRE_B:
			Serial3.println("###FBW B\t2***");
			break;
		case AUTO:
			Serial3.println("###AUTO\t6***");
			break;
		case RTL:
			Serial3.println("###RTL\t7***");
			break;
		case LOITER:
			Serial3.println("###LOITER\t8***");
			break;
		case TAKEOFF:
			Serial3.println("##TAKEOFF\t9");
			break;
		case LAND:
			Serial3.println("##LAND\t10");
			break;
	}
}

void print_position2EMU(void)
{	
   // Serial.println("!!!LAT:48563884,LON:2871495,ALT:102,ALH:167,CRS:148,BER:149,XTR:0,WPN:1,DST:204,FIX:0,***");
   /*
        current_loc.lat=485638840;
        current_loc.lng=28714950;
        current_loc.alt=10000;

           */
        Serial.print("!!!");
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
	Serial.print(g.waypoint_index,DEC);//Actually is the waypoint.
	Serial.print(",DST:");
	Serial.print((int)wp_distance,DEC);
	Serial.print(",FIX:");
	Serial.print(g_gps->fix,DEC);
	Serial.println(",***");
}

void printPerfData(void)
{
}

void print_attitude2ADS(void)
{  
      output_ADS();
  /*
    	Serial3.println("");
 	Serial3.print("+++ASP:");
	Serial3.print(ground_speed/100,DEC);
	Serial3.print(",TTH:");
	Serial3.print(g.channel_throttle.servo_out,DEC);
	Serial3.print (",RLL:");
	Serial3.print(nav_roll/100,DEC);
	Serial3.print (",PCH:");
	Serial3.print(nav_pitch/100,DEC);
	Serial3.println(",***");
*/
	
}

void output_gpsemulator(void)
{   
    //Serial.println("!!!STT:1,WPN:1,DST:204,RER:0,PSET:3,THH:54,***");
 
    Serial.print("!!!");
    Serial.print ("STT:");
    if((control_mode==AUTO)||(control_mode==LOITER)) Serial.print("1");
    else if((control_mode==MANUAL)||(control_mode==STABILIZE)) Serial.print("0");
    else Serial.print((int)readSwitch());
    Serial.print (",WPN:");
    Serial.print((int)g.waypoint_index,DEC);  //Actually is the waypoint.
    Serial.print (",DST:");
    Serial.print((int)wp_distance,DEC);
    Serial.print (",RER:");
    Serial.print(nav_roll/100,DEC);
    Serial.print (",PSET:");
    Serial.print(nav_pitch/100,DEC);
    Serial.print(",THH:");
    Serial.print(g.channel_throttle.servo_out,DEC);
    Serial.println(",***");
}

void output_ADS(void)
{   
    //Serial.println("!!!STT:1,WPN:1,DST:204,RER:0,PSET:3,THH:54,***");

    Serial3.print("Sw:"); Serial3.print((int)readSwitch());
    Serial3.print("   Roll:"); Serial3.print(nav_roll/100,DEC);
    Serial3.print("  Pitch:"); Serial3.print(nav_pitch/100,DEC);
    Serial3.print("  Nav_Bearing:"); Serial3.print(nav_bearing/100,DEC);
    Serial3.print("   Throttle:"); Serial3.println(g.channel_throttle.servo_out,DEC); 
    Serial3.print("Control mode:"); Serial3.println(control_mode,DEC);
/*
    Serial3.print("!!!");
    Serial3.print ("STT:");
    Serial3.print((int)readSwitch());
    Serial3.print (",WPN:");
    Serial3.print((int)g.waypoint_index,DEC);  //Actually is the waypoint.
    Serial3.print (",DST:");
    Serial3.print((int)wp_distance,DEC);
    Serial3.print (",RER:");
    Serial3.print((int)(nav_roll/100),DEC);
    Serial3.print (",PSET:");
    Serial3.print((int)(nav_pitch/100),DEC);
    Serial3.print(",THH:");
    Serial3.print((int)g.channel_throttle.servo_out,DEC);
    Serial3.println(",***");
*/
}

void print_position2ADS(void)
{       

        if (home_is_set) ready2nav = 1;
        
        print_control_mode();
            
        Serial3.print("Hom Pos: Lat:"); Serial3.print(home.lat/10,DEC);
	Serial3.print("  Lon:"); Serial3.print(home.lng/10,DEC);
        Serial3.print("  Alt:"); Serial3.println(home.alt/100,DEC);
        Serial3.print("Cur Pos: Lat:"); Serial3.print(current_loc.lat/10,DEC);
	Serial3.print("  Lon:"); Serial3.print(current_loc.lng/10,DEC);
        Serial3.print("  Alt:"); Serial3.println(current_loc.alt/100,DEC);
        Serial3.print("Prev Wp  Lat:"); Serial3.print(prev_WP.lat/10,DEC);
	Serial3.print("  Lon:"); Serial3.print(prev_WP.lng/10,DEC);
        Serial3.print("  Alt:"); Serial3.println(prev_WP.alt/100,DEC);
        Serial3.print("Newt Wp  Lat:"); Serial3.print(next_WP.lat/10,DEC);
	Serial3.print("  Lon:"); Serial3.print(next_WP.lng/10,DEC);
        Serial3.print("  Alt:"); Serial3.print(next_WP.alt/100,DEC);
        Serial3.print("  Num:"); Serial3.println(g.waypoint_index,DEC);
        Serial3.print("SPD:"); Serial3.print(ground_speed/100,DEC);
	Serial3.print("  ALR:"); Serial3.print((int)(altitude_error/100),DEC);
	Serial3.print("  HDG:"); Serial3.print(ground_course/100,DEC);
 	Serial3.print("  TAR:"); Serial3.print(target_bearing/100,DEC);
        Serial3.print("  DST:"); Serial3.print(wp_distance,DEC);
   	Serial3.print("  XTR:"); Serial3.print((int)crosstrack_error,DEC);     
   	Serial3.print("  Berr:"); Serial3.println(bearing_error/100,DEC);     
   	Serial3.println("-------------------");
   if (back2home == 1) Serial3.println("---- BACK TO HOME ----");
    else if (back2home == 2) Serial3.println("---- CLOSED LOOP NAV ----");
/*
	Serial3.println("");
        Serial3.print("!!!LAT:");
	Serial3.print(current_loc.lat/10,DEC);
	Serial3.print(",LON:");
	Serial3.print(current_loc.lng/10,DEC); //wp_current_lat
	Serial3.print(",CRS:");
	Serial3.print(ground_course/100,DEC);
	Serial3.print(",SPD:");
	Serial3.print(ground_speed/100,DEC);		
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
	Serial3.print(",BTV:12.34");
	Serial3.print(",MOD:");
	Serial3.print(control_mode,DEC);
	Serial3.print(",FIX:");
	Serial3.print(ready2nav);		 // 1 = ready2nav
	Serial3.println(",***");
*/
}

#endif
