// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID_fast.cpp
/// @brief	Generic PID algorithm

#include <math.h>

#include "PID_fast.h"

long
PID_fast::get_pid(int32_t error, float dt)
{
	// Compute proportional component
	_output = error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0)) {

		_derivative = (error - _last_error) / dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
		        (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_error 		= error;
		_last_derivative    = _derivative;

		// add in derivative component
		_output 	+= _kd * _derivative;
	}

	// Compute integral component if time has elapsed
	if ((fabs(_ki) > 0) && (dt > 0)) {
		_integrator 		+= (error * _ki) * dt;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		_output 	+= _integrator;
	}

	return _output;
}

void
PID_fast::reset_I()
{
	_integrator = 0;
	_last_error = 0;
	_last_derivative = 0;
}

void
PID_fast::load_gains()
{
    _group.load();
}

void
PID_fast::save_gains()
{
    _group.save();
}
