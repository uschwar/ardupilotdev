/*
 * GNU GPL
 *
 * 2008 binarymillenium
 *
 *
 */

#include <stdio.h>
//#include <sys/socket.h>
//#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
//#include <netinet/in.h>
#include <errno.h>
#include <time.h>

#include <cmath>

#include <iostream>
#include <fstream>

//pour Windows
#include <winsock2.h>
#include <windows.h> 

typedef signed int       int32_t;
typedef unsigned int     uint32_t;

#include "net_fdm.hxx"
#include "net_ctrls.hxx"

#include "fgrxtx.hpp"
#include "auto_float.hpp"

#define MAXBUFLEN 1024

signed int float_to_fixed(double val)
{
	return (signed int)(val*FIX1);
}

signed int float_to_fixed16(double val)
{
	return (signed int)(val*(1<<24));
}

double fixed_to_float(signed int fixed_val)
{
	return (float)(fixed_val)/(float)(FIX1);
}

double fixed_to_float16(signed int fixed_val)
{
	return (float)(fixed_val)/(float)(1<<24);
}


void state_fixed_to_float(known_state_ints& state_ints, known_state& state)
{
	state.p = fixed_to_float(state_ints.p);
	state.q = fixed_to_float(state_ints.q);
	state.r = fixed_to_float(state_ints.r);
	
	state.dq = fixed_to_float(state_ints.dq);
	state.dr = fixed_to_float(state_ints.dr);
	state.iq = fixed_to_float(state_ints.iq);
	state.ir = fixed_to_float(state_ints.iq);
	
	state.error_heading = fixed_to_float(state_ints.error_heading);
	state.derror_heading = fixed_to_float(state_ints.derror_heading);
	state.ierror_heading = fixed_to_float(state_ints.ierror_heading);
	
	state.error_pitch = fixed_to_float(state_ints.error_pitch);
	state.dpitch = fixed_to_float(state_ints.dpitch);
	state.ipitch = fixed_to_float(state_ints.ipitch);
}


static void htond (double &x)   
{
		int    *Double_Overlay;
		int     Holding_Buffer;

		Double_Overlay = (int *) &x;
		Holding_Buffer = Double_Overlay [0];

		Double_Overlay [0] = htonl (Double_Overlay [1]);
		Double_Overlay [1] = htonl (Holding_Buffer);
}


static void htonf (float &x)
{
//	if ( sgIsLittleEndian() ) {
		int    *Float_Overlay;
		int     Holding_Buffer;

		Float_Overlay = (int *) &x;
		Holding_Buffer = Float_Overlay [0];

		Float_Overlay [0] = htonl (Holding_Buffer);
//	} else {
//		return;
//	}
}

void FGProps2NetCtrls( FGNetCtrls *net)
{
	int i = 0;

	// convert to network byte order
	net->version = htonl(net->version);
	htond(net->aileron);
	htond(net->elevator);
	htond(net->rudder);
	htond(net->aileron_trim);
	htond(net->elevator_trim);
	htond(net->rudder_trim);
	htond(net->flaps);
	net->flaps_power = htonl(net->flaps_power);
	net->flap_motor_ok = htonl(net->flap_motor_ok);

	net->num_engines = htonl(net->num_engines);
	for ( i = 0; i < FGNetCtrls::FG_MAX_ENGINES; ++i ) {
		net->master_bat[i] = htonl(net->master_bat[i]);
		net->master_alt[i] = htonl(net->master_alt[i]);
		net->magnetos[i] = htonl(net->magnetos[i]);
		net->starter_power[i] = htonl(net->starter_power[i]);
		htond(net->throttle[i]);
		htond(net->mixture[i]);
		net->fuel_pump_power[i] = htonl(net->fuel_pump_power[i]);
		htond(net->prop_advance[i]);
		htond(net->condition[i]);
		net->engine_ok[i] = htonl(net->engine_ok[i]);
		net->mag_left_ok[i] = htonl(net->mag_left_ok[i]);
		net->mag_right_ok[i] = htonl(net->mag_right_ok[i]);
		net->spark_plugs_ok[i] = htonl(net->spark_plugs_ok[i]);
		net->oil_press_status[i] = htonl(net->oil_press_status[i]);
		net->fuel_pump_ok[i] = htonl(net->fuel_pump_ok[i]);
	}

	net->num_tanks = htonl(net->num_tanks);
	for ( i = 0; i < FGNetCtrls::FG_MAX_TANKS; ++i ) {
		net->fuel_selector[i] = htonl(net->fuel_selector[i]);
	}

	net->cross_feed = htonl(net->cross_feed);
	htond(net->brake_left);
	htond(net->brake_right);
	htond(net->copilot_brake_left);
	htond(net->copilot_brake_right);
	htond(net->brake_parking);
	net->gear_handle = htonl(net->gear_handle);
	net->master_avionics = htonl(net->master_avionics);
	htond(net->wind_speed_kt);
	htond(net->wind_dir_deg);
	htond(net->turbulence_norm);
	htond(net->temp_c);
	htond(net->press_inhg);
	htond(net->hground);
	htond(net->magvar);
	net->icing = htonl(net->icing);
	net->speedup = htonl(net->speedup);
	net->freeze = htonl(net->freeze);
}

void FGNetFDM2Props( FGNetFDM *net ) {
    unsigned int i;

    //if ( net_byte_order ) {
        // Convert to the net buffer from network format
        net->version = ntohl(net->version);

        htond(net->longitude);  // use
        htond(net->latitude);   // use 
        htond(net->altitude);   // use
        htonf(net->agl);
        htonf(net->phi);
        htonf(net->theta);
        htonf(net->psi);
        htonf(net->alpha);
        htonf(net->beta);

		///  custom- put ifdef around?
        htonf(net->p);		// use
        htonf(net->q);	// use
        htonf(net->r); 	// use


        htonf(net->phidot);		// use
        htonf(net->thetadot);	// use
        htonf(net->psidot); 	// use
        htonf(net->vcas);
        htonf(net->climb_rate);
        htonf(net->v_north);
        htonf(net->v_east);
        htonf(net->v_down);
        htonf(net->v_wind_body_north);
        htonf(net->v_wind_body_east);
        htonf(net->v_wind_body_down);

        htonf(net->A_X_pilot);	// use
        htonf(net->A_Y_pilot);  // use
        htonf(net->A_Z_pilot);	// use

        htonf(net->stall_warning);
        htonf(net->slip_deg);

}


int main(void)
{
	int sock, sock2;
	struct sockaddr_in echoclient,from, ctrlclient, ctrlinclient;
	unsigned int echolen, clientlen;
	int received = 0;

	FGNetFDM buf;
	char* buffer = (char*)&buf;

	FGNetCtrls bufctrl;
	char* buffer_ctrl = (char*)&bufctrl; 

	struct known_state state; 
	struct known_state old_state; 

	struct known_state_ints state_ints; 
	struct known_state_ints old_state_ints; 
	
	srand ( time(NULL) );
	memset(&state,		0, sizeof(state));
	memset(&old_state,	0, sizeof(state));

	memset(&state_ints,		0, sizeof(state_ints));
	memset(&old_state_ints,	0, sizeof(state_ints));


	/* Create the UDP socket */
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		perror("socket");
		return -1;
	}

	if ((sock2 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		perror("socket");
		return -1;
	}

	/// the port where net fdm is received
	memset(&echoclient, 0, sizeof(echoclient));       /* Clear struct */
	echoclient.sin_family = AF_INET;                  /* Internet/IP */
	echoclient.sin_addr.s_addr = inet_addr("127.0.0.1");  /* IP address */
	echoclient.sin_port = htons(5500);       /* server port */

	/// the port where the modified net ctrl is sent to flightgear
	memset(&ctrlclient, 0, sizeof(ctrlclient));       /* Clear struct */
	ctrlclient.sin_family = AF_INET;                  /* Internet/IP */
	ctrlclient.sin_addr.s_addr = inet_addr("127.0.0.1");  /* IP address */
	ctrlclient.sin_port = htons(5600);       /* server port */

	/// this is the port where the net ctrl comes from flight gear
	memset(&ctrlinclient, 0, sizeof(ctrlinclient));       /* Clear struct */
	ctrlinclient.sin_family = AF_INET;                  /* Internet/IP */
	ctrlinclient.sin_addr.s_addr = inet_addr("127.0.0.1");  /* IP address */
	ctrlinclient.sin_port = htons(5700);       /* server port */

	int length = sizeof(buf);

	std::cout << "packet length should be " << length << " bytes" << std::endl;

	echolen = length;

	char errorbuf[100];

	/// need this?
	int ret  = bind(sock,  (struct sockaddr*)&echoclient, sizeof(echoclient) );
	int ret2 = bind(sock2, (struct sockaddr*)&ctrlinclient, sizeof(ctrlinclient) );
	
	std::cout << "bind " << ret << " " << ret2 << std::endl;
	///////////////////////////////////////////////////////////////////

	std::string file = "telemetry.csv";
	std::ofstream telem(file.c_str(), std::ios_base::out );
	if (!telem) {
		std::cout << "File \"" << file << "\" not found.\n";
		throw fileNotFound();
		return 2;
	}
	telem.precision(10);

    telem <<
	        "longitude, latitude, altitude" <<
			        "p, q, r," <<
					"elevator, rudder, aileron" <<
							            std::endl;

	//std::string binfile = "sim_telemetry.bin";
	//std::ofstream myFile(binfile, std::ios::out | std::ios::binary);

	double time = 0.0; 

	float dist = 40e3; //state.altitude;
	float div = 3e7;


	double target_longitude = -2.137; 
	double target_latitude  = .658;
	double target_altitude  = 50.0; //meters 
	std::cout << "struct size " << sizeof(state) << std::endl;

	FILE* telem_file;
	telem_file = fopen("telem.bin","wb");

	int i = 0;
	while(1) {
	 
	 	i++;
		// I think this clobbers echoclient
		/// receive net_fdm over udp
		received = recvfrom(sock, buffer, sizeof(buf), 0,
						(struct sockaddr *) &from,
						&echolen);
	
/// -extract accelerations A_X_pilot etc, add noise
/// - are phidot, thetadot, psidot in body frame? Probably 
/// (I thought someone told me there was no difference, but that doesn't 
/// make sense- what if the vehicle was 90 degrees up from normal, how is
/// roll still roll?)
/// - take position lat long alt, but filter so it is only updated at a few Hz

/// and that's all we'll know

		FGNetFDM2Props( &buf);

		if (i == 1) {
			target_longitude = buf.longitude + (float)(rand()%(int)dist)/div - dist/div*0.5; 
			target_latitude  = buf.latitude  + (float)(rand()%(int)dist)/div - dist/div*0.5; 
			std::cout << "t longitude=" << target_longitude << ", t latitude=" << target_latitude << std::endl;
		}

		if (i%10 == 0) {
			state.longitude= (float)((int)(180e4/M_PI*buf.longitude))/(180e4/M_PI);;
			state.latitude = (float)((int)(180e4/M_PI*buf.latitude))/(180e4/M_PI);
			state.altitude = buf.altitude*M2FT;
		} else {
			state.longitude= old_state.longitude;
			state.latitude = old_state.latitude;
			state.altitude = old_state.altitude;
		}

		state.p = FPFIX(buf.p);
		state.q = FPFIX(buf.q);
		state.r = FPFIX(buf.r);

		state.A_X_pilot = FPFIX(buf.A_X_pilot);
		state.A_Y_pilot = FPFIX(buf.A_Y_pilot);
		state.A_Z_pilot = FPFIX(buf.A_Z_pilot);

		state.target_long= target_longitude;
		state.target_lat = target_latitude;
		state.target_alt = target_altitude;
		
		#if 0
		/// fixed point version
		if (j%10 == 0) {
		    /// convert to arc-minutes
			state_ints.longitude = float_to_fixed(180.0*60.0/M_PI*buf.longitude);
			state_ints.latitude  = float_to_fixed(180.0*60.0/M_PI*buf.latitude);
			/// convert to feet later?
			state_ints.altitude  = (int)(buf.altitude);

			/// TEMP
			/// alternate way of getting velocity- this one matches the sim 
			double x1 = (EARTH_RADIUS_METERS+state.altitude)*cos(state.latitude)*cos(state.longitude);
			double y1 = (EARTH_RADIUS_METERS+state.altitude)*cos(state.latitude)*sin(state.longitude);
			double z1 = (EARTH_RADIUS_METERS+state.altitude)*sin(state.latitude);

			double x2 = (EARTH_RADIUS_METERS+old_state.altitude)*cos(old_state.latitude)*cos(old_state.longitude);
			double y2 = (EARTH_RADIUS_METERS+old_state.altitude)*cos(old_state.latitude)*sin(old_state.longitude);
			double z2 = (EARTH_RADIUS_METERS+old_state.altitude)*sin(old_state.latitude);

			// delta xyz to target
			double xt = (EARTH_RADIUS_METERS+target_altitude)*cos(target_latitude)*cos(target_longitude);
			double yt = (EARTH_RADIUS_METERS+target_altitude)*cos(target_latitude)*sin(target_longitude);
			double zt = (EARTH_RADIUS_METERS+target_altitude)*sin(target_latitude);

			double dx = (x1-x2);
			double dy = (y1-y2);
			double dz = (z1-z2);

			// length of vector pointing from old location to current
			float l1 = sqrtf(dx*dx + dy*dy + dz*dz);

			/// length of vector point straight at center of earth
			double l2 = sqrtf(x2*x2 + y2*y2 + z2*z2);
		
			std::cout << "velocity correct " << l1
				<< ", dx " << (dx);
			//	<< ", radius " << EARTH_RADIUS_METERS 
			//	<< ", coslat " << cos(state.latitude)*cos(state.longitude);
				//<< ", dy " << (y1)
				//<< ", dz " << (z1);
			/*
			double dotprod = ( 
								(dx/l1 * (-x2)/l2) +
								(dy/l1 * (-y2)/l2) +
								(dz/l1 * (-z2)/l2)  );
			
			state.pitch = acos(dotprod)/M_PI -0.5;
			*/
		}
		j++;

		state_ints.target_long= float_to_fixed(target_longitude);
		state_ints.target_lat = float_to_fixed(target_latitude);
		state_ints.target_alt = float_to_fixed(target_altitude);

		state_ints.p = float_to_fixed(buf.p);
		state_ints.q = float_to_fixed(buf.q);
		state_ints.r = float_to_fixed(buf.r);

		state_ints.A_X_pilot = float_to_fixed(buf.A_X_pilot);
		state_ints.A_Y_pilot = float_to_fixed(buf.A_Y_pilot);
		state_ints.A_Z_pilot = float_to_fixed(buf.A_Z_pilot);
#endif

		if (received < 0) {
			perror("recvfrom");
	//	} else if (received != echolen) {
	//		std::cerr << "wrong sized packet " << received << std::endl;
		} else {

			/// get the ctrls that are properly filled out with environmental
			/// data, so we don't clobber that when we modify the flap positions
			received = recvfrom(sock2, buffer_ctrl, sizeof(bufctrl), 0,
						(struct sockaddr *) &ctrlinclient,
						&echolen);

			FGProps2NetCtrls(&bufctrl);

			float dt =1.0/10.0;
			autopilot(state, old_state,dt);
			bufctrl.elevator = FPFIX(state.elevator);
			bufctrl.aileron  = FPFIX(state.aileron);
			bufctrl.rudder   = FPFIX(state.rudder);

			//autopilot_ints(state_ints, old_state_ints, buf, bufctrl);
			//state_fixed_to_float(state_ints,state);

			/// output to file and stdout 
			{
				/// save telemetry to file
				telem << 
					buf.longitude << "," << buf.latitude << "," << buf.altitude*M2FT << "," <<
					state.p << "," << state.q << "," << state.r << "," <<
					bufctrl.elevator << "," << bufctrl.rudder << "," << bufctrl.aileron << "," <<
					state.dq << "," << state.iq << "," << 
					state.error_heading << "," << state.derror_heading << "," << state.ierror_heading << "," << 
					state.error_pitch << "," << state.dpitch << "," << state.ipitch << "," <<
					state.tpitch << "," << state.speed << "," << 
					state.tdx << "," << state.tdy << "," << 
					bufctrl.wind_speed_kt << "," << bufctrl.wind_dir_deg << "," << bufctrl.press_inhg << "," <<  
					state.dr << "," << state.ir << "," << state.pitch << 
					state.A_X_pilot << "," << state.A_Y_pilot << "," << state.A_Z_pilot << "," << 
					state.longitude << "," << state.latitude << "," << state.altitude << "," <<
					std::endl;


				/// output some of the state to stdout every few seconds
				static int i = 0;
				i++;
				std::cout.precision(2);
				//if ((state.longitude != old_state.longitude ) || (state.latitude != old_state.latitude)) {
				if (i%30==0) {
					std::cout <<
						", hor dist=" << sqrtf(state.tdist*state.tdist - (state.altitude-target_altitude)*(state.altitude-target_altitude)) <<
						//", agl=" << buf.agl << ", alt=" << state.altitude << ", vel=" << state.speed << 
						//		", fdm v= " << sqrtf(buf.v_north*buf.v_north + buf.v_east*buf.v_east + buf.v_down*buf.v_down)*0.3048 <<
						//	" tdx=" << state.tdx << ", tdy=" << state.tdy <<
						//		" heading= " << heading << " target heading= " << theading <<
						", err_head= " << state.error_heading << ", rud=" << bufctrl.rudder <<
						//			", lat= " << state.latitude << ", long= " << state.longitude << ", alt= " << state.altitude <<
						", pitch= " << state.pitch << ", tpitch= " << state.tpitch << ", elev=" << bufctrl.elevator <<
						//	", p=" << state.p <<  
						//		", q=" << state.q << ", elev=" << bufctrl.elevator <<
						", r=" << state.r << ", ail= " << bufctrl.aileron << 
						", ax=" << state.A_X_pilot << ", ay=" << state.A_Y_pilot << ", az=" << state.A_Z_pilot << 
						", orient=" << state.acc_orientation <<
						std::endl;
				}
			}

			fwrite((void*)&state, 1, sizeof(state), telem_file);

			//std::cout << old_state.altitude << " " << state.altitude << std::endl;
			memcpy(&old_state, &state, sizeof(state));
			
			//memcpy(&old_state_ints, &state_ints, sizeof(state_ints));


			////////////////////////////////////////////////////////////////////////

			received = sendto(sock, buffer_ctrl, sizeof(bufctrl), 0,
			(struct sockaddr *) &ctrlclient, sizeof(ctrlclient));

			
			if (received <0) {
				perror("sendto");
			}

			time += 0.1;	
		
		}

		usleep(3000);
	}

	fclose(telem_file);

return 0;
}
