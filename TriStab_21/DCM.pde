// DCM Code based on ArduIMU DCM from Jordi Munoz and William Premerlani
//
// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
// --------------------------------------------------------------

void Normalize(void)
{
	float error = 0;
	float temporary[3][3];
	float renorm = 0;
	boolean problem = FALSE;
	
	error= -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; // eq.19

	Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); // eq.19
	Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); // eq.19
	
	Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]); //eq.19
	Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]); //eq.19
	
	Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b // eq.20
	
	renorm = Vector_Dot_Product(&temporary[0][0], &temporary[0][0]); 
	if (renorm < 1.5625f && renorm > 0.64f) {				// Check if we are OK with Taylor expansion
		renorm = .5 * (3 - renorm);					// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1. / sqrt(renorm);													
		renorm_sqrt_count++;
	} else {
		problem = TRUE;
		renorm_blowup_count++;
	}
	Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
	
	renorm = Vector_Dot_Product(&temporary[1][0], &temporary[1][0]); 
	if (renorm < 1.5625f && renorm > 0.64f) {				// Check if we are OK with Taylor expansion
		renorm = .5 * (3 - renorm);					// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1. / sqrt(renorm);	
		renorm_sqrt_count++;
	} else {
		problem = TRUE;
		renorm_blowup_count++;
	}
	Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
	
	renorm = Vector_Dot_Product(&temporary[2][0], &temporary[2][0]); 
	if (renorm < 1.5625f && renorm > 0.64f) {				// Check if we are OK with Taylor expansion
		renorm = .5 * (3 - renorm);					// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1. / sqrt(renorm);	
		renorm_sqrt_count++;
	} else {
		problem = TRUE;	
		renorm_blowup_count++;
	}
	Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
	
	if (problem) {					// Our solution is blowing up and we will force back to initial condition.
		DCM_Matrix[0][0] = 1.0f;
		DCM_Matrix[0][1] = 0.0f;
		DCM_Matrix[0][2] = 0.0f;
		DCM_Matrix[1][0] = 0.0f;
		DCM_Matrix[1][1] = 1.0f;
		DCM_Matrix[1][2] = 0.0f;
		DCM_Matrix[2][0] = 0.0f;
		DCM_Matrix[2][1] = 0.0f;
		DCM_Matrix[2][2] = 1.0f;
		problem = FALSE;	
	}
}

/**************************************************/
void Drift_correction(void)
{
	//Compensation the Roll, Pitch and Yaw drift. 
	float mag_heading_x;
	float mag_heading_y;
	float errorCourse = 0;
	static float Scaled_Omega_P[3];
	static float Scaled_Omega_I[3];
	float Accel_magnitude;
	float Accel_weight;
	float Integrator_magnitude;
	static bool in_motion = 0;
	
	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	Accel_magnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
	Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.

	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1);	//	
	
	//	We monitor the amount that the accelerometer based drift correction is deweighted for performanc reporting
	imu_health = imu_health + 0.02 * (Accel_weight-.5);
		imu_health = constrain(imu_health, 0, 1);
	
	Vector_Cross_Product(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); // adjust the ground of reference 
	// errorRollPitch are in Accel ADC units
	// Limit max errorRollPitch to limit max Omega_P and Omega_I
	errorRollPitch[0] = constrain(errorRollPitch[0], -50, 50);
	errorRollPitch[1] = constrain(errorRollPitch[1], -50, 50);
	errorRollPitch[2] = constrain(errorRollPitch[2], -50, 50);

	Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);
	
	Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);		 
	
	//*****YAW***************
	
	#if MAGNETOMETER == 1 
		// We make the gyro YAW drift correction based on compass magnetic heading
		errorCourse= (DCM_Matrix[0][0] * APM_Compass.Heading_Y) - (DCM_Matrix[1][0] * APM_Compass.Heading_X);	// Calculating YAW error	
	#else	// Use GPS Ground course to correct yaw gyro drift=
		if(GPS.ground_speed >= SPEEDFILT){
			// Optimization: We have precalculated COGX and COGY (Course over Ground X and Y) from GPS info
			errorCourse = (DCM_Matrix[0][0] * COGY) - (DCM_Matrix[1][0] * COGX);	// Calculating YAW error
				
		} else {
			errorCourse = 0;
		}
	#endif
	
	Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse); // Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW);
	Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding	Proportional.

	Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW);
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I	 
	

	//	Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
	Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I, Omega_I));
	if (Integrator_magnitude > ToRad(300)) {
		Vector_Scale(Omega_I, Omega_I, 0.5f * ToRad(300) / Integrator_magnitude);
	}
}

/**************************************************/
void Accel_adjust(void)
{
   Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
   Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 

// Accel_Vector[1] += Accel_Scale((GPS.ground_speed / 100) * Omega[2]);	// Centrifugal force on Acc_y = GPS_speed * GyroZ
// Accel_Vector[2] -= Accel_Scale((GPS.ground_speed / 100) * Omega[1]);	// Centrifugal force on Acc_z = GPS_speed * GyroY 
}

/**************************************************/
void Matrix_update(void)
{
	static float filt[3] = {0.0,0.0,0.0};
	
	Gyro_Vector[0] = Gyro_Scaled_X(read_adc(0)); // gyro x roll
	Gyro_Vector[1] = Gyro_Scaled_Y(read_adc(1)); // gyro y pitch
	Gyro_Vector[2] = Gyro_Scaled_Z(read_adc(2)); // gyro Z yaw
	
	//Record when you saturate any of the gyros.
	if((abs(Gyro_Vector[0]) >= ToRad(300)) || (abs(Gyro_Vector[1]) >= ToRad(300)) || (abs(Gyro_Vector[2]) >= ToRad(300)))
			gyro_sat_count++;

	//Accel_Vector[0]=read_adc(3); // acc x
	//Accel_Vector[1]=read_adc(4); // acc y
	//Accel_Vector[2]=read_adc(5); // acc z 
	// Low pass filter on accelerometer data (to filter vibrations)
	filt[0] = filt[0] * 0.6 + (float)read_adc(3) * 0.4; // acc x
	filt[1] = filt[1] * 0.6 + (float)read_adc(4) * 0.4; // acc y
	filt[2] = filt[2] * 0.6 + (float)read_adc(5) * 0.4; // acc z
	Accel_Vector[0] = filt[0];
	Accel_Vector[1] = filt[1];
	Accel_Vector[2] = filt[2];

	Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);	// adding Integrator term
	Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); // adding proportional term

	Accel_adjust();		// Remove centrifugal acceleration.
	
 #if OUTPUTMODE == 1				 
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; // -z
	Update_Matrix[0][2] = G_Dt * Omega_Vector[1]; // y
	Update_Matrix[1][0] = G_Dt * Omega_Vector[2]; // z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; // -x
	Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; // -y
	Update_Matrix[2][1] = G_Dt * Omega_Vector[0]; // x
	Update_Matrix[2][2] = 0;
 #else										// Uncorrected data (no drift correction)
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; // -z
	Update_Matrix[0][2] = G_Dt * Gyro_Vector[1]; // y
	Update_Matrix[1][0] = G_Dt * Gyro_Vector[2]; // z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
	Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
	Update_Matrix[2][1] = G_Dt * Gyro_Vector[0];
	Update_Matrix[2][2] = 0;
 #endif

	Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); // a * b = c

	for(int x = 0; x < 3; x++){ // Matrix Addition (update)
		for(int y = 0; y < 3; y++){
			DCM_Matrix[x][y] += Temporary_Matrix[x][y];
		} 
	}

}

/**************************************************/
void Euler_angles(void)
{
	#if (OUTPUTMODE == 2)				 // Only accelerometer info (debugging purposes)
		roll 			= atan2(Accel_Vector[1], Accel_Vector[2]);		// atan2(acc_y, acc_z)
		pitch 			= -asin((Accel_Vector[0]) / (double)GRAVITY); // asin(acc_x)
		yaw 			= 0;
	#else
		pitch 			= -asin(DCM_Matrix[2][0]);
		roll 			= atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
		yaw 			= atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
	#endif
 
}

