// ************************************************************************* 
//                  TriStab, an IMU stabilizer for a Tricopter                 
//                   An Open Source Arduino based multicopter.
// Author : Updated and improved version by Jean-Louis Naudin for Tricopter
// More info at: http://diydrones.com/profile/JeanLouisNaudin
// TrIMUpter blog: http://diydrones.com/profiles/blogs/the-trimupter-a-vtol-tricopter
// -------------------------------------------------------------------------

// Debug options - set only one of these options to 1 at a time, set the others to 0
#define DEBUG_SUBSYSTEM 0   	// -1 = no debug output
                                //  0 = debug Roll, pitch, yaw, K_aux, magnetic heading and GPS datas
								//  1 = Debug print the Radio and IMU parameters
                                //  2 = Debug Stick inputs from Rx Vs PID errors
                                //  3 = Debug the GPS
                                //  4 = Debug the IMU sensors
                                //  5 = Debug PID
                                
//------------------------- USER PARAMETERS --------------------------------

// TriCopter PID GAINS        // IN/OUTDOOR ( Tested in flights )
#define KP_TRI_ROLL 1.1       //   1.1    
#define KD_TRI_ROLL 0.45      //   0.45    
#define KI_TRI_ROLL 0.15      //   0.15   

#define KP_TRI_PITCH 1.0      //   1.0    
#define KD_TRI_PITCH 0.40     //   0.40   
#define KI_TRI_PITCH 0.15     //   0.15   

#define KP_TRI_YAW 3          //   3.8    
#define KD_TRI_YAW 2.0        //   2.0   
#define KI_TRI_YAW 0.1        //   0.18    

// Position control PID GAINS ( Tested in flights 11-15-10)
/*
#define  KP_GPS_ROLL 0.015
#define  KI_GPS_ROLL 0.005
#define  KD_GPS_ROLL 0.01
#define  KP_GPS_PITCH 0.015
#define  KI_GPS_PITCH 0.005
#define  KD_GPS_PITCH 0.01
*/
// Position control PID GAINS ( Tested in flights 11-16-10) // FULL AUTONOMOUS GPS OK
#define  KP_GPS_ROLL 0.030
#define  KI_GPS_ROLL 0.008
#define  KD_GPS_ROLL 0.03
#define  KP_GPS_PITCH 0.030
#define  KI_GPS_PITCH 0.008
#define  KD_GPS_PITCH 0.03

#define  GPS_MAX_ANGLE 22 // Maximun command roll and pitch angle from position control

// Position altitude PID GAINS
#define  KP_ALTITUDE 0.8
#define  KI_ALTITUDE 0.2
#define  KD_ALTITUDE 0.2

#define ROLL_PITCH_GAIN 18.71   // With greater value the Tricopter is more responsive to Roll/Pitch stick command (tested value:18)

#define KNOB_TUNING  1 // tuning of the ROLL_PITCH_GAIN gain with knob on ch6   1:Yes, 0: No

//Adjust this parameter for your lattitude
#define  GEOG_CORRECTION_FACTOR 0.6578  // cos(lattitude) Paris=48.8666°

// Local magnetic declination
// I use this web : http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp
#define local_declination 1.85 // Magnetic declination in Paris (France)

#define MAGNETOMETER 1       // 0 : No magnetometer        1: Magnetometer HMC5843
#define HEADING_LOCK 0       // 0 : Inertial Heading Lock  1: Magnetic Heading Lock

#define MAX_CHANNELS  9      // Number of radio channels to read (7 is the number if you use the PPM encoder from store.diydrones.com)

#define IsGPS          // Do we have a GPS connected
//#define IsNEWMTEK    // Do we have MTEK with new firmware

#define GPS_PROTOCOL 2    // 1 - NMEA,  2 - EM406,  3 - Ublox, 4 - MediaTek  

/* Setup of the Graupner MX16s Transmitter for the TrIMUpter
Stick mode: 2, Modulation: PPM, Mode: Fixed-Wing
Tail type: normal, AL/Flaps: 1 Aileron
Receiver Output: S2->Output 1, S3->Output 2, S1->Output 3, S4->Output 4, S5->Output 5
Servos setting: S1 => 0% 125% 125%, S2 <= 0% 100% 100%, S3 <= 0% 100% 100%, S4 <= 0% 100% 100%, S5 => 0% 100% 100%
Controls setting: E5 ^2 +100% +100%
Dual Rate/Expo: AL 100% 0%, PR 100% 0%, DI 100% 0%
*/

#define MIN_YAW    1100       // Min Max value for the YAW servo output
#define MAX_YAW    1900

#define MAX_ROLL_ANGLE 30   // Max angle in deg for Roll
#define MAX_PITCH_ANGLE 30  // Max angle in deg for Pitch

#define MAX_CONTROL_OUTPUT 250  // Max/min auto-control to servo

#define MIN_THROTTLE 1011       // Throttle pulse width at minimun...
#define CHANN_CENTER 1500
#define DEAD_CENTER  15

#define SPEKTRUM 0  // Spektrum radio

// DCM PID GAINS
#define Kp_ROLLPITCH 0.0014	 		// Pitch&Roll Drift Correction Proportional Gain
#define Ki_ROLLPITCH 0.0000003 		        // Pitch&Roll Drift Correction Integrator Gain
#define Kp_YAW 0.8		 		// Yaw Drift Correction Porportional Gain	
#define Ki_YAW 0.00004 				// Yaw Drift CorrectionIntegrator Gain

/*Min Speed Filter for Yaw drift Correction*/
#define SPEEDFILT 3 // >1 use min speed filter for yaw drift cancellation, 0=do not use

/*For debugging purposes*/
#define OUTPUTMODE 1  //If value = 1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 Accel only data

#define GPS_CONNECTION 0 // 0 for GPS pins, 1 for programming pins

// PPM Rx signal read (ICP) constants and variables
#define Servo1Pin   9             // Servo pins...
#define Servo2Pin   10
#define icpPin	    8             // Input Capture Pin (Rx signal reading)

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
int SENSOR_SIGN[]={1,-1,-1,1,-1,1}; //{1,-1,-1,1,-1,1} (org values)

// The IMU should be correctly adjusted : Gyro Gains and also initial IMU offsets:
// We have to take this values with the IMU flat (0º roll, 0ºpitch)
// AN[0]:365.00, AN[1]:372.05, AN[2]:376.83, AN[3]:505.00, AN[4]:504.00, AN[5]:504.00

float acc_offset_x=505;
float acc_offset_y=504;
float acc_offset_z=504;
float gyro_offset_roll=370;  
float gyro_offset_pitch=373;
float gyro_offset_yaw=380;

#define AUTOset 0 // if 1 Accelerometer autosetup at the startup else 0 hard coded values below

