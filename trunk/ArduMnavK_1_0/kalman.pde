/*
 *  Based on:
 *
 *  kalman.c by Filipe Varela on 06/04/20.
 *  Copyright 2006 Filipe Varela. All rights reserved.
 *
 *		You are free to do whatever you want with this code except:
 *		You may not release derivates without crediting rotomotion, "The Global Positioning System and Inertial Navigation" and this work.
 *		You may not release derivates without including full source code of libraries, executables or modules that directly use this work.
 *
 *	All code custom writen except:
 *		Kalman algorythm for state_update and kalman_update from rotomotion's tilt.c, credit included in file
 *		Cross axis interference processing matrix from "The Global Positioning System and Inertial Navigation" ISBN: XXX XXXXXXX
 *      This book takes care of the missing part in tilt.c which is multiaxis and cross-axis interference
 *
 */

#define r2d 57.29577951308	// = 180 / PI
#define d2r 00.01745329252	// = PI / 180
#define PI 3.1415926535
#define r2hex 163		// = 256 / (pi/2)

#include <math.h>

//state variables
float gyroRate[3]; 	//gyro measured: roll, nick, yaw in rad/sec
float gyroBias[3];	// bias is the second
float accelVal[3]; 	//Ax (=roll),Ay (=nick), Az

//kalman estimates
float angleMeasured[3];	 			//accelerometer based angle (measured)
float angleEstimate[3]; 			// angle is the first part of the state vector

float dt = (1.0 / 10);		// update funcs are called every 100ms

int tempDisp;			// for debugging purposes

/*
 * Our Process covariance matrix.  This is updated at every time step to
 * determine how well the sensors are tracking the actual state.
 */

float P[3][2][2] = {
	{
		{ 1, 0 },
		{ 0, 1 },
	},
	{
		{ 1, 0 },
		{ 0, 1 },
	},
	{
		{ 1, 0 },
		{ 0, 1 },
	}
};

/*
 * R represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect R rad jitter
 * from the accelerometer.
 */
const float R_angle = 	0.2;

/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 */
const float Q_angle = 	0.05;		// = angle noise E(alpha^2)
const float Q_gyro = 	0.0001;		// = bias noise  E(bias^2)

int	totalG;			// total acceleration = sqrt(x^2 + y^2 + z^2)
int	gX, gY, gZ;		// static acceleration without angle offsets

float gyroCal[3];		// only for gyro calibration
int   calN;
int   calR;
int   calY;

/*
 * kalman reset is called when sensors are calibrated
*/
void kalmanReset()
{
	uint8_t i;

	for (i=0; i<3; i++)
	{
		angleMeasured[i] = 0.0;
		angleEstimate[i] = 0.0;
		gyroBias[i] = 0.0;

		P[i][0][0] = 1;
		P[i][0][1] = 0;
		P[i][1][0] = 0;
		P[i][1][1] = 1;

		gyroCal[i]=0.0;
	}
  Serial.println("*** Kalman reset ***");
}

/*
 * readSensors is called every 10ms. It transforms the raw gyro rates into radians/dt
 * rescales the accels values and computes the nick and roll angles in degrees*4. These are
 * then used in the mixer.
*/
void readSensors (void)
{
//	float sR,tN, cR		; //, cN;
	
	gyroRate[0] = (float)p;  // in rad/s
	gyroRate[1] = (float)q;  // in rad/s
	gyroRate[2] = (float)r;  // in rad/s

	accelVal[0] = (float)ax; // in m/s^2
	accelVal[1] = (float)ay; // in m/s^2
	accelVal[2] = (float)az; // in m/s^2

	angleN = angleEstimate[0]*r2d;		// covert to degrees 
	angleR = angleEstimate[1]*r2d;

	calN = (int)(gyroCal[1]* r2hex);
	calR = (int)(gyroCal[0]* r2hex);
	calY = (int)(gyroCal[2]* r2hex);

	/***************************************************************************
		ESTIMATOR MATRIX
		---------------------------------------
		[phi_dot	]	[1	sin(phi)tan(theta)	cos(phi)*tan(theta)	]	[P]
		[theta_dot	] =     [0	cos(phi)		-sin(phi)	 	] *	[Q]
		[psi_dot	]	[0	sin(phi)/cos(theta)	cos(phi)/cos(theta)	]	[R]
	***************************************************************************/
	
//	sR = sin(angleEstimate[0]);
//	cR = cos(angleEstimate[0]);
//	cN = cos(angleEstimate[1]);
//	tN = tan(angleEstimate[1]);
	
//	angleRate[0] = gyroRate[0]              + 	gyroRate[1]*sR*tN 	+ 	gyroRate[2]*cR*tN;
//	angleRate[1] = gyroRate[1]*cR 		+ 	gyroRate[2]*-sR;
//	angleRate[2] = gyroRate[1]*(sR/cN)      + 	gyroRate[2]*(cR/cN);

}

/*
 * accel2angle computes the angles with the accel values
*/
void accel2angle(void)
{
	float f;
	
	// Calculate g ( ||g_vec|| )
	float g = sqrt(accelVal[0]*accelVal[0] + accelVal[1]*accelVal[1] + accelVal[2]*accelVal[2]);
	
	totalG = (g - 1.0) * 64;	// = |a| - gravity

	//values in radians
	angleMeasured[0] = -asin(accelVal[0] / g);
	angleMeasured[1] = asin(accelVal[1] / g);
	angleMeasured[2] = 0;	// later: magnetometer ...

	/* try to compute the linear part of the acceleration.
	   We assume, that the angleEstimate currently are the correct angles.
	*/
	f = 64 * (accelVal[0] + sin(angleEstimate[0]));
	gX = (int)f;
	f = 64 * (sin(angleEstimate[1]) - accelVal[1]);
	gY = (int)f;
	gZ = (int)(totalG - gX - gY);

	tempDisp = gZ;
}

void state_update() //called on new gyro output available
{
	uint8_t i;
	
	/*
	 * Compute the derivative of the covariance matrix
	 *
	 *	Pdot = A*P + P*A' + Q
	 *	  
	 	Pdot = ( 0,-1)*(p(0,0), p(0,1)) + (p(0,0), p(0,1))*( 0, 0) + (Qa, 0)
			   ( 0, 0) (p(1,0), p(1,1))   (p(1,0), p(1,1)) (-1, 0)   ( 0,Qg)

			 = (-p(1,0), -p(1,1)) +  (-p(0,1), 0) + (Qa, 0)
			   (   0   ,    0   )    (-p(1,1), 0)   ( 0,Qg)

			 = (Qa - p(1,0) - p(0,1)  , -p(1,1) )
	 		   ( -p(1,1)              ,    Qg   )    
	 */

	for(i=0;i<2;i++)			// set limit to 3 when Yaw can be measured
	{
		// 	integrate angle by adding corrected gyro rate
		angleEstimate[i] += (dt * (gyroRate[i] - gyroBias[i]));

		gyroCal[i] += (dt * gyroRate[i]);

		float Pdot[2 * 2] = {
			Q_angle - P[i][0][1] - P[i][1][0],	/* 0,0 */
					- P[i][1][1],				/* 0,1 */
					- P[i][1][1],				/* 1,0 */
			Q_gyro								/* 1,1 */
		};

		/* Update the covariance matrix */
		P[i][0][0] += Pdot[0] * dt;
		P[i][0][1] += Pdot[1] * dt;
		P[i][1][0] += Pdot[2] * dt;
		P[i][1][1] += Pdot[3] * dt;
	}

	gyroCal[2] += (dt * gyroRate[2]);
}

void kalmanFilter(void)
{
	float angle_err;
	uint8_t i;
	
	const float C_0 = 1.0;		// our extractor matrix. We only want the angle part, not the bias
	
	for(i=0;i<2;i++)			// set limit to 3 when Yaw can be measured
	{
		angle_err = angleMeasured[i] - angleEstimate[i];

		const float PCt_0 = C_0 * P[i][0][0];
		const float PCt_1 = C_0 * P[i][1][0];
			
		//error estimate
		const float E = R_angle + C_0 * PCt_0;

		const float	K_0 = PCt_0 / E;
		const float	K_1 = PCt_1 / E;
			
		const float	t_0 = PCt_0;
		const float	t_1 = C_0 * P[i][0][1];

		P[i][0][0] -= K_0 * t_0;
		P[i][0][1] -= K_0 * t_1;
		P[i][1][0] -= K_1 * t_0;
		P[i][1][1] -= K_1 * t_1;
		
		angleEstimate[i] += K_0 * angle_err;
		gyroBias[i] += K_1 * angle_err;
	}
}

