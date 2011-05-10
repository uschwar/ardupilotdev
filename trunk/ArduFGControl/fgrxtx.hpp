#ifndef FGRXTX_HPP
#define FGRXTX_HPP

#include <cmath>
#include <iostream>
#include <fstream>

#include "net_fdm.hxx"
#include "net_ctrls.hxx"

/// doesn't work if beyond a single t away
#define CLAMP(x,t) {if ((x) > (t)) (x) = (t);  if ((x) < -(t)) (x) = -(t); }

const double start_longitude = -2.1355;
const double start_latitude = 0.656384;

struct known_state_ints
{
	
	signed int longitude;
	signed int latitude;
	signed int altitude;

	signed int target_lat;  /// degrees in 12bit fixed
	signed int target_long; /// degrees in 12bit fixed
	signed int target_alt;  /// feet in standard int

	/// distance from target
	/// this should actually be distance from fixed 0 point since target might move
	signed int tdx;
	signed int tdy;

	signed int heading;
	signed int target_heading;

	signed int error_heading; /// degrees
	signed int derror_heading; /// filtered
	signed int ierror_heading;

	signed int pitch;
	
	signed int error_pitch;
	signed int dpitch;  // these are for error_pitch
	signed int ipitch;

	signed int tpitch;  // target pitch
	signed int speed;  // ft/s
	signed int tdist;  // direct distance to target

	signed int p;
	signed int q;
	signed int r;

	signed int dq; // derivative of q (filtered)
	signed int iq; // integral of q

	signed int dr;
	signed int ir;

	signed int elevator;

	signed int A_X_pilot;
	signed int A_Y_pilot;
	signed int A_Z_pilot;

};

class fileNotFound {};

// lose precision
#define FPFIX(x) fixed_to_float(float_to_fixed(x))
/// 12 bits of fraction precision
const signed int FIX1 = 1<<10;

signed int float_to_fixed(double val);
signed int float_to_fixed16(double val);
double fixed_to_float(signed int fixed_val);
double fixed_to_float16(signed int fixed_val);

void autopilot_ints(known_state_ints& state, known_state_ints& old_state,
					FGNetFDM& buf, FGNetCtrls& bufctrl);
#endif

