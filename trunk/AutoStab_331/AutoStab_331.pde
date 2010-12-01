// ************************************************************************* 
//                  AutoStab, an IMU stabilizer for a VTOL UAV                 
//                   An Open Source Arduino based multicopter.
// Authors : Updated and improved version by Jean-Louis Naudin and Matthieu Bourdarias
//           from the French VTOL/UAV Team
// -------------------------------------------------------------------------
// The Main Code use some parts of Jose Julio code
// The DCM Code is based on ArduIMU DCM from Jordi Munoz and William Premerlani
/* 
     This program is free software: you can redistribute it and/or modify 
     it under the terms of the GNU General Public License as published by 
     the Free Software Foundation, either version 3 of the License, or 
     (at your option) any later version. 
 
      This program is distributed in the hope that it will be useful, 
      but WITHOUT ANY WARRANTY; without even the implied warranty of 
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
      GNU General Public License for more details. 
 
      You should have received a copy of the GNU General Public License 
      along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
// ************************************************************************* 
// More info at: http://diydrones.com/profile/JeanLouisNaudin
// Scorpid-500 OAT Project: http://diydrones.com/profiles/blogs/the-scorpid500-uav-an-oat
// IMU Hardware : 
//   ArduIMU+ v2 flat (6DOF) (sparkfun)
//   HMC 5843 Triple axis magnetometer (sparkfun)
//   EM406 SirfIII GPS (GlobalSat) (sparkfun)
// - 1 receiver Graupner R16scan (with the serial PPM output directly taken into the receiver)
// - 1 transmitter Graupner MX16s
// -------------------------------------------------------------------------
// Project start date : 10-28-10
// Version : 3.2 with GPS position Hold feature and OAT UAV special mixers
// Last update: Nov 26, 2010 - JLN
//
// 10-28-10:  JLN - Tricopter CCPM 120° mixer
// 10-31-10:  JLN - Automatic offset for accelerometers
// 11-01-10:  JLN - The CCPM mixer works fine 
// 11-02-10:  JLN - Fine tuning of the CCPM mixer in FLIGHT... Its Flight now !!!
// 11-03-10:  JLN - Tested successfully in (flight indoor/outdoor)
// 11-04-10:  JLN - Outdoor tests flights in windy conditions (10 Kts, gusting 15 Kts)
// 11-04-10:  JLN - Increase the Yaw stick command gain from 18 to 25 for a better response
// 11-06-10:  JLN - Correction of bug about the Yaw drift during pitch and roll (DCM)
// 11-07-10:  JLN - Now support the APM_Compass library, Yaw, Pitch and Roll are stable and the Magnetic Heading true...
// 11-07-10:  JLN - Successfull test in flight of the v1.3
// 11-07-10:  JLN - Now adding AP_GPS library for the GPS position hold feature v1.3 -> v2.0
// 11-08-10:  JLN - GPS is working, tested with EM406, GPS position hold is activated with the SW2 connected on ch5
// 11-09-10:  JLN - The PID gain can be tuned with the knob (54%) connected on ch6 of the transmitter
// 11-09-10:  JLN - Redesign of the Heading lock system for the Yaw control
// 11-10-10:  JLN - The Heading lock on the Yaw works well and heading is locked when yaw stick is released
// 11-10-10:  JLN - Successful tests in flights: IMU stabilisation, Heading Lock, GPS position hold are OK
// 11-10-10:  JLN - v2.1 released to public on: http://code.google.com/p/ardupilotdev/downloads/list
// 11-21-10:  JLN - Now a new version v3.0 with a special mixer for the VTOL/OAT Scorpid-500 UAV
// 11-22-10:  JLN - SUCCESSFUL TEST FLIGHT with the SCORPID-500 OAT UAV ( Roll only IMU stabilisation test flight ) mixer1 is OK
// 11-22-10:  JLN - v3.2 added the Yaw/Pitch (hold) feature on  mixer2 - Ok working in lab now but not yet tested in flight
// 11-23-10:  JLN - v3.2 tested in flight, the OAT mixer1 and mixer2 are OK and works very well
// 11-23-10:  JLN - v3.31 Minor modif on Pitch hold and Pitch stick response
// 11-23-10:  JLN - v3.31 Minor change to be compatible with the latest version of the APM_Compass library 
// 11-26-10:  JLN - v3.31 New tested in flight, Pitch seems now OK...

// ToDo List: 
// - adding the autonomous flights under flight plan with the ArduPilot board used as the navigation controller and datalogger (TriStab v3)

#include <inttypes.h>
#include <math.h>
#include <FastSerial.h>		// ArduPilot Fast Serial Library
#include <AP_GPS.h>			// ArduPilot GPS library

#include <Wire.h>           // For magnetometer readings
#include <APM_Compass.h>    // Compass Library

#include "userparms.h"

/* Software version */
#define VER 3.31    // Current software version (only numeric values)

APM_Compass_Class APM_Compass;

/* ***************************************************************************************** */
// ADC : Voltage reference 3.3v / 10bits(1024 steps) => 3.22mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 3.22mV/ADC step => 330/3.22 = 102.48
// Tested value : 101
#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// LPR530 & LY530 Sensitivity (from datasheet) => 3.33mV/º/s, 3.22mV/ADC step => 1.03
// Tested values : 0.96,0.96,0.94
#define Gyro_Gain_X 0.92 //X axis Gyro gain
#define Gyro_Gain_Y 0.92 //Y axis Gyro gain
#define Gyro_Gain_Z 0.94 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define TRUE 1
#define FALSE 0

#define vers "AutoStab, an autopilot for the OAT-UAV SCORPID-500 Jean-Louis Naudin - v "

#define SYNC_GAP_LEN    8000      // we assume a space at least 4000us is sync (note clock counts in 0.5 us ticks)
#define MIN_IN_PULSE_WIDTH (750)  //a valid pulse must be at least 750us (note clock counts in 0.5 us ticks)
#define MAX_IN_PULSE_WIDTH (2250) //a valid pulse must be less than  2250us

// PPM Rx signal read END

// Servo Timer2 variables (Servo Timer2)
#define SERVO_MAX_PULSE_WIDTH 2000
#define SERVO_MIN_PULSE_WIDTH 900
#define SERVO_TIMER2_NUMSERVOS 4         // Put here the number of servos. In this case 4 ESC´s

uint8_t sensors[6] = {6,7,3,0,1,2};  // For Hardware v2 flat

//Position control
long target_longitude;
long target_lattitude;
byte target_position;
float gps_err_roll;
float gps_err_roll_old;
float gps_roll_D;
float gps_roll_I=0;
float gps_err_pitch;
float gps_err_pitch_old;
float gps_pitch_D;
float gps_pitch_I=0;
float command_gps_roll;
float command_gps_pitch;

float AN[6];                 //array that store the 6 ADC filtered data
float AN_OFFSET[6];          //Array that stores the Offset of the gyros

float G_Dt=0.01;             // Integration time for the gyros (DCM algorithm)
long  GPS_timer;
long  GPS_timer_old;
float GPS_Dt=0.2;   // GPS Dt
float Mag_Dt=0.1;   // MAG Dt

float Accel_Vector[3]= {0,0,0};             //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0,0,0};  //Store the acceleration in a vector
float Accel_magnitude;
float Accel_weight;
float Gyro_Vector[3]= {0,0,0};              //Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0};             //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};                  //Omega Proportional correction
float Omega_I[3]= {0,0,0};                  //Omega Integrator
float Omega[3]= {0,0,0};

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
float errorCourse=0;
float COGX=0;                               //Course overground X axis
float COGY=0;

float roll=0;
float pitch=0;
float yaw=0;

//Altitude control
int Initial_Throttle;
int err_altitude;
int err_altitude_old;
float command_altitude;
float altitude_I;
float altitude_D;

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

// Startup GPS variables
int gps_fix_count = 5;		//used to count 5 good fixes at ground startup

// GPS Selection
FastSerialPort0(Serial);		// Instantiate the fast serial driver
#if   GPS_PROTOCOL == 1
AP_GPS_NMEA		GPS(&Serial);
#elif GPS_PROTOCOL == 2
AP_GPS_406		GPS(&Serial);
#elif GPS_PROTOCOL == 3
AP_GPS_UBLOX	GPS(&Serial);
#elif GPS_PROTOCOL == 4
AP_GPS_MTK		GPS(&Serial);
#else
# error Must define GPS_PROTOCOL with a valid value.
#endif

// Performance monitoring
// ----------------------
long 	perf_mon_timer;
float 	imu_health; 							// Metric based on accel gain deweighting
int 	G_Dt_max;								// Max main loop cycle time in milliseconds
byte 	gyro_sat_count;
byte 	adc_constraints;
byte 	renorm_sqrt_count;
byte 	renorm_blowup_count;
byte	gcs_messages_sent;

static volatile unsigned int Pulses[ MAX_CHANNELS + 1];  // Pulse width array
static volatile uint8_t  Rx_ch = 0;
int Neutral[MAX_CHANNELS+1];    // neutral values for each Rx channel in case of fail safe...
byte radio_status=0;            // radio_status = 1 => OK, 0 => No Radio signal
byte hmc5843ok=0;

static volatile unsigned int ICR1_old = 0;
static volatile unsigned int Timer1_last_value;  // to store the last timer1 value before a reset

 // Rx Channel values
int ch1;       // Ailerons
int ch2;       // Elevator
int ch3;       // Throttle
int ch4;       // Rudder
// Servo Channel values
int chESC1;    
int chESC2;
int chSERVO1;
int chSERVO2;
int ch_aux;
int ch_aux2;

int ch1_old;    // Old channel values
int ch2_old;
int ch3_old;
int ch4_old;
int ch_aux_old;
int ch_aux2_old;

typedef struct {
  uint8_t pin;
  int value;
  uint8_t counter;
}  servo_t;

uint8_t num_servos=SERVO_TIMER2_NUMSERVOS;
servo_t Servos[SERVO_TIMER2_NUMSERVOS];

static volatile uint8_t Servo_Channel;
static volatile uint8_t ISRCount=0;
static volatile unsigned int Servo_Timer2_timer1_start;
static volatile unsigned int Servo_Timer2_timer1_stop;
static volatile unsigned int Servo_Timer2_pulse_length;
// Servo Timer2 variables END 

// Servo variables (OC1 and OC2) for standard servos [disabled in this version]
unsigned int Servo1;
unsigned int Servo2;

float speed_3d=0; //Speed (3-D)

union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

// ADC variables
volatile uint8_t MuxSel=0;
volatile uint8_t analog_reference = DEFAULT;
volatile uint16_t analog_buffer[8];
volatile uint8_t analog_count[8];
int an_count;

// Attitude control variables
float Stick_cmd_roll=0;        // stick commands from the receiver
float Stick_cmd_roll_old;
float Stick_cmd_roll_diff;
float Stick_cmd_pitch=0;
float Stick_cmd_pitch_old;
float Stick_cmd_pitch_diff;
float Stick_cmd_yaw=0;
float Stick_cmd_yaw_old;
float Stick_cmd_yaw_diff;
float TargetHeading;

int control_roll;           // order sent to servos
int control_pitch;
int control_yaw;
int hlock;

float K_aux;
float roll_I=0;
float roll_D;
float err_roll;
float err_roll_ant;
float pitch_I=0;
float pitch_D;
float err_pitch;
float err_pitch_ant;
float yaw_I=0;
float yaw_D;
float err_yaw;
float err_yaw_ant;

// AP_mode : 1=> GPS position hold  2=>Stabilization normal mode
byte AP_mode = 2;             

long t0;
int num_iter;
float aux_debug;

// ************************************************************ 
// ROLL, PITCH and YAW PID controls...
// ************************************************************ 
// Input : desired Roll, Pitch and Yaw absolute angles. Output : Motor commands
void Attitude_control_stable()
{

#if KNOB_TUNING == 0   
  K_aux=PITCH_GAIN; // tuning of the ROLL_PITCH_GAIN gain with knob on ch6   1:Yes, 0: No
#endif 
  
// ROLL CONTROL   ============================================  
  err_roll_ant = err_roll;
  if (AP_mode==2)        // NORMAL Stabilization mode
    err_roll = Stick_cmd_roll - ToDeg(roll);
  else
   err_roll = (Stick_cmd_roll + command_gps_roll) - ToDeg(roll);  // GPS Position control   
   
  err_roll = constrain(err_roll,-MAX_ROLL_ANGLE,MAX_ROLL_ANGLE);  // to limit max roll command...
  
  roll_I += err_roll*G_Dt;
  roll_I = constrain(roll_I,-50,50);
  // D term implementation => two parts: gyro part and command part
  // To have a better (faster) response we can use the Gyro reading directly for the Derivative term...
  // Omega[] is the raw gyro reading plus Omega_I, so it´s bias corrected
  // We also add a part that takes into account the command from user (stick) to make the system more responsive to user inputs

   roll_D = Stick_cmd_roll_diff*ROLL_GAIN - ToDeg(Omega[0]);  // Take into account Angular velocity of the stick (command)
  
  // PID control for ROLL
  control_roll = KP_TRI_ROLL*err_roll + KD_TRI_ROLL*roll_D + KI_TRI_ROLL*roll_I; 
  control_roll = constrain(control_roll,-MAX_CONTROL_ROLL_OUTPUT,MAX_CONTROL_ROLL_OUTPUT);
  
// PITCH CONTROL ==============================================
  err_pitch_ant = err_pitch;
  if (AP_mode==2)        // NORMAL Stabilization mode
    err_pitch = Stick_cmd_pitch - ToDeg(pitch);
  else
    err_pitch = (Stick_cmd_pitch + command_gps_pitch) - ToDeg(pitch);  // GPS Position Control
    
  err_pitch = constrain(err_pitch,-MAX_PITCH_ANGLE,MAX_PITCH_ANGLE);  // to limit max pitch command...
  
  pitch_I += err_pitch*G_Dt;
  pitch_I = constrain(pitch_I,-150,150);
  // D term
  pitch_D = Stick_cmd_pitch_diff - ToDeg(Omega[1]);
 
  // PID control for PITCH
  control_pitch = KP_TRI_PITCH*err_pitch + KD_TRI_PITCH*pitch_D + KI_TRI_PITCH*pitch_I; 
  control_pitch = constrain(control_pitch,-MAX_CONTROL_PITCH_OUTPUT,MAX_CONTROL_PITCH_OUTPUT);
  
#if DEBUG_SUBSYSTEM == 5  
  // PID analysis and debug  
  Serial.print("ch2:"); Serial.print(ch2); 
  Serial.print("   Stick_cmd_pitch("); Serial.print(Stick_cmd_pitch);
  Serial.print(") - pitch("); Serial.print(ToDeg(pitch));
 
  Serial.print(") => err_pitch="); Serial.print(err_pitch); 
  
  Serial.print(",  pitch_I=");  Serial.print(pitch_I); 
//  Serial.print(", Omega[1]=");   Serial.print(ToDeg(Omega[1])); 
  Serial.print(",  pitch_D=");   Serial.print(pitch_D);
  Serial.print(",  contral_pitch=");   Serial.print(control_pitch);
  Serial.print(",  K_aux=");   Serial.print(K_aux);
  Serial.print(",  PITCH_GAIN=");   Serial.println(PITCH_GAIN);
#endif
  
// YAW CONTROL WITH HEADING LOCK (if magnetometer installed) =================================
  
     // Proportional term
    err_yaw_ant = err_yaw; 
    
#if HEADING_LOCK == 1    
    if (hmc5843ok==1) err_yaw = TargetHeading - ToDeg(APM_Compass.Heading);  // Magnetic based heading lock         
    else err_yaw = TargetHeading - ToDeg(yaw);                               // Inertial based heading lock
#else
    err_yaw = TargetHeading - ToDeg(yaw);                               // Inertial based heading lock
#endif

    if (err_yaw > 180)    // Normalize to -180,180
      err_yaw -= 360;
    else if(err_yaw < -180)
      err_yaw += 360;   
 // err_yaw = constrain(err_yaw,-60,60);  // to limit max yaw command... 
  
  yaw_I += err_yaw*G_Dt;                                           // Integral term
  yaw_I = constrain(yaw_I,-60,60);
  
  yaw_D = Stick_cmd_yaw_diff - ToDeg(Omega[2]);                   // Derivative term
 
  // PID control for YAW
  control_yaw = KP_TRI_YAW*err_yaw + KD_TRI_YAW*yaw_D + KI_TRI_YAW*yaw_I;
  control_yaw = constrain(control_yaw,-MAX_CONTROL_YAW_OUTPUT,MAX_CONTROL_YAW_OUTPUT);  

}

/* ************************************************************ */
/* GPS based Position control */
void Position_control(long lat_dest, long lon_dest)
{
  long Lon_diff;
  long Lat_diff;

  Lon_diff = lon_dest - GPS.longitude;
  Lat_diff = lat_dest - GPS.latitude;

  // ROLL
  //Optimization : cos(yaw) = DCM_Matrix[0][0] ;  sin(yaw) = DCM_Matrix[1][0] 
  gps_err_roll = (float)Lon_diff * GEOG_CORRECTION_FACTOR * DCM_Matrix[0][0] - (float)Lat_diff * DCM_Matrix[1][0];

  gps_roll_D = (gps_err_roll-gps_err_roll_old) / GPS_Dt;
  gps_err_roll_old = gps_err_roll;
  
  gps_roll_I += gps_err_roll * GPS_Dt;
  gps_roll_I = constrain(gps_roll_I, -800, 800);

  command_gps_roll = KP_GPS_ROLL * gps_err_roll + KD_GPS_ROLL * gps_roll_D + KI_GPS_ROLL * gps_roll_I;
  command_gps_roll = constrain(command_gps_roll, -GPS_MAX_ANGLE, GPS_MAX_ANGLE); // Limit max command
  
  // PITCH
  gps_err_pitch = -(float)Lat_diff * DCM_Matrix[0][0] - (float)Lon_diff * GEOG_CORRECTION_FACTOR * DCM_Matrix[1][0];

  gps_pitch_D = (gps_err_pitch - gps_err_pitch_old) / GPS_Dt;
  gps_err_pitch_old = gps_err_pitch;

  gps_pitch_I += gps_err_pitch * GPS_Dt;
  gps_pitch_I = constrain(gps_pitch_I, -800, 800);

  command_gps_pitch = KP_GPS_PITCH * gps_err_pitch + KD_GPS_PITCH * gps_pitch_D + KI_GPS_PITCH * gps_pitch_I;
  command_gps_pitch = constrain(command_gps_pitch, -GPS_MAX_ANGLE, GPS_MAX_ANGLE); // Limit max command

}
// ************************************************************ 
int channel_filter(int ch, int ch_old)
{
  int diff_ch_old;

  if (ch_old==0)      // ch_old not initialized
    return(ch);
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old < 0)
  {
    if (diff_ch_old <- 60)
      return(ch_old - 60);        // We limit the max difference between readings
  }
  else
  {
    if (diff_ch_old > 60)    
      return(ch_old + 60);
  }
  return((ch + ch_old) >> 1);   // Small filtering
  //return(ch);
}

long timer=0; //general porpuse timer 
long timer_old;

//  ******************************** Setup *****************
void setup(){
  
  int i, j;
  int aux;
 
  Serial.begin(38400, 128, 16);
  pinMode(2,OUTPUT); //Serial Mux
  if (GPS_CONNECTION == 0){
    digitalWrite(2,HIGH); //Serial Mux
  } else {
    digitalWrite(2,LOW); //Serial Mux
  }

  pinMode(8,INPUT);       // Rx Radio Input
  pinMode(5,OUTPUT);      // Red LED
  pinMode(6,OUTPUT);      // BLue LED
  pinMode(7,OUTPUT);      // Yellow LED
  pinMode(9,OUTPUT);      // Servo5
  pinMode(10,OUTPUT);     // Servo1  Right ESC
  pinMode(11,OUTPUT);     // Servo2  Left  ESC
  pinMode(12,OUTPUT);     // Servo3  Right Servo 
  pinMode(13,OUTPUT);     // Servo4  Left Servo  
 
  ch1=CHANN_CENTER;      // Aileron
  ch2=CHANN_CENTER;      // Elevator
  ch3=MIN_THROTTLE;      // Throttle
  ch4=CHANN_CENTER;      // Rudder
  ch_aux=CHANN_CENTER;   // Aux
  ch_aux2=CHANN_CENTER;  // Aux2 
 
  delay(100);
  Stick_cmd_yaw = 0;
  Servo1 = 1500;
  Servo2 = 1500;
     
#ifdef IsGPS  
  GPS.init();                // GPS Initialization
#ifdef IsNEWMTEK  
  delay(250);
  // DIY Drones MTEK GPS needs binary sentences activated if you upgraded to latest firmware.
  // If your GPS shows solid blue but LED C (Red) does not go on, your GPS is on NMEA mode
  Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n"); 
#endif
#endif

  digitalWrite(6,HIGH); // BLUE Led
  digitalWrite(5,HIGH); // RED Led
  Serial.begin(38400);
  Serial.println();
  Serial.print(vers);
  Serial.println(VER);
#ifdef IsGPS  
  Serial.println("*** GP_Init OK");  
#endif    
  RxServoInput_ini();  // Servos input initialisation routines
  delay(1000);
   
 // Take neutral radio values...
 for (j=1;j<=6;j++)
   Neutral[j] = RxGetChannelPulseWidth(j);

 for (i=0; i<80; i++)
   {
   for (j=1;j<=6;j++)
     Neutral[j] = (Neutral[j]*0.8 + RxGetChannelPulseWidth(j)*0.2);
   delay(25);
   }
 Serial.println("Check Radio"); 
 Serial.print("ch1:");  Serial.print(Neutral[1]);
 Serial.print(",ch2:"); Serial.print(Neutral[2]);
 Serial.print(",ch3:"); Serial.print(Neutral[3]);
 Serial.print(",ch4:"); Serial.print(Neutral[4]);
 Serial.print(",ch5:"); Serial.print(Neutral[5]);
 Serial.print(",ch6:"); Serial.println(Neutral[6]);
 
 // Roll, Pitch and Throttle have fixed neutral values (the user can trim the radio)
 #if SPEKTRUM==1
   Neutral[1] = MIN_THROTTLE;
 #else
   Neutral[3] = MIN_THROTTLE;
 #endif

 // Assign pins to servos
 num_servos = 4;
 Servos[0].pin = 10;       // Right ESC
 Servos[1].pin = 11;       // LEFT ESC
 Servos[2].pin = 12;       // Right servo
 Servos[3].pin = 13;       // Left servo
 Servo_Timer2_set(0,MIN_THROTTLE);   // First assign values to servos
 Servo_Timer2_set(1,MIN_THROTTLE);
 Servo_Timer2_set(2,Neutral[2]);
 Servo_Timer2_set(3,Neutral[4]);
 Servo_Timer2_ini();                // Servo Interrupt initialization
 
 chESC1=MIN_THROTTLE;    // Right ESC
 chESC2=MIN_THROTTLE;    // LEFT ESC
 chSERVO1=Neutral[2];    // Right servo
 chSERVO2=Neutral[4];    // Left servo

 Analog_Reference(EXTERNAL);
 Analog_Init();
 
 // Magnetometer initialization
#ifdef MAGNETOMETER == 1
    Serial.println("Check HMC5843 magnetometer");
    APM_Compass.Init();  // I2C initialization

    APM_Compass.SetOrientation(MAGORIENTATION);
    APM_Compass.SetOffsets(MAGOFFSET);
    APM_Compass.SetDeclination(ToRad(DECLINATION));
    
    // Check the health of magnetometer
    APM_Compass.Read();                // call the APM_Compass library routines
    if ((APM_Compass.Mag_X==-32)&&(APM_Compass.Mag_Y==32)&&(APM_Compass.Mag_Z==-32)) 
    { Serial.println("*** WARNING --- MAG INIT ERROR ---");
      for (j=1;j<=30;j++)
      {      digitalWrite(7,HIGH);  // Yellow Led
             digitalWrite(6,LOW);   // Blue Led
             digitalWrite(5,HIGH);  // Red Led
             delay(500);
             digitalWrite(7,LOW);   // Yellow Led
             digitalWrite(6,HIGH);  // Blue Led
             digitalWrite(5,LOW);   // Red Led
             delay(500);
             hmc5843ok = 0;
      }
    }
    else  
    { Serial.println("*** MAG INIT OK");
      hmc5843ok = 1;
    }
#endif
    delay(1000);
    
 Read_adc_raw();  // read analog values of the IMU sensors
 delay(20);

 // Offset values for accels and gyros...
 AN_OFFSET[3] = acc_offset_x;
 AN_OFFSET[4] = acc_offset_y;
 AN_OFFSET[5] = acc_offset_z;
 AN_OFFSET[0] = gyro_offset_roll;
 AN_OFFSET[1] = gyro_offset_pitch;
 AN_OFFSET[2] = gyro_offset_yaw;

 // Take the gyro offset values
 for(int i=0;i<600;i++)
    {  digitalWrite(5,HIGH);   // Blue Led
    Read_adc_raw();
    for(int y=0; y<=2; y++)   // Read initial ADC values for offset.
      AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
    delay(20);
    digitalWrite(5,LOW);      // Blue Led
    }
    
#if AUTOset==1
   Get_ACC_Offset();  // used in case of automatic offset at the startup, need to be horizontal
#endif 
 
 digitalWrite(5,HIGH);    // Red Led
 delay(200);
 
 // Wait until throttle stick is at bottom
 #if SPEKTRUM==1
 while (RxGetChannelPulseWidth(1)>(MIN_THROTTLE+50)){
 #else
 while (RxGetChannelPulseWidth(3)>(MIN_THROTTLE+50)){
 #endif
    digitalWrite(7,LOW);     // Yellow Led
    digitalWrite(6,HIGH);    // Blue Led
    digitalWrite(5,LOW);     // Red Led
   Serial.println("Move throttle stick to bottom to MINIMUM !!!");
   Serial.print("Radio Channels:");
   for (i=1;i<=MAX_CHANNELS;i++)
     {
     Serial.print(RxGetChannelPulseWidth(i));
     Serial.print(",");
     }
   Serial.println();
   delay(50);
   digitalWrite(7,HIGH);   // Yellow Led
   digitalWrite(6,LOW);    // Blue Led
   digitalWrite(5,HIGH);   // Red Led
   delay(50);
 }

 Read_adc_raw();   // Start ADC readings...
 timer = millis();
 delay(20);
 digitalWrite(6,LOW);   // Blue Led
 digitalWrite(5,LOW);  // Red Led
 digitalWrite(7,HIGH); // Yellow Led

 Serial.println("*** Ready to fly ***");
}

// ******************************** Main Loop *****************
void loop(){
  
  int aux;
  float aux_float, aux_float2;
  
  if((millis()-timer)>=14)   // 14ms => 70 Hz loop rate 
  { counter++;
    timer_old = timer;
    timer=millis();
    G_Dt = (timer-timer_old)/1000.0f;      // Real time of loop run: G_Dt=0.01 sec
    num_iter++;

#if MAGNETOMETER == 1    
    if (G_Dt*counter >= Mag_Dt)  // Read compass data at 10Hz...
      { counter=0;
       if (hmc5843ok==1)   // if magnetic data are valid only
          { APM_Compass.Read();                // call the APM_Compass library routines
            APM_Compass.Calculate(roll, pitch);
          }
      }
#endif

    read_AHRS(); // Get gyro and accel data and perform IMU calculations

 
    // *****************
    // Telemetry data...
     
    if (num_iter>5)
    { num_iter=0;
#if DEBUG_SUBSYSTEM == 0    
//    Serial.print("G_Dt:");            Serial.print(G_Dt);
      Serial.print("\tR:");             Serial.print(ToDeg(roll));
      Serial.print("\tP:");             Serial.print(ToDeg(pitch));
      Serial.print("\tY:");             Serial.print(ToDeg(yaw));
      Serial.print("\t\tK_aux:");       Serial.print(K_aux);
      Serial.print("\tHdg:");             Serial.print(ToDeg(APM_Compass.Heading));   
                                // display the output for the servos
//    Serial.print("\t\tThr:");          Serial.print(ch3);
//    Serial.print("\t\tAil:");          Serial.print(ch1);                                
    Serial.print("\t\tEscR:");         Serial.print(chESC1);
    Serial.print("\tEscL:");           Serial.print(chESC2);
    Serial.print("\tSrvR:");           Serial.print(chSERVO1);
    Serial.print("\tSrvL:");           Serial.println(chSERVO2);    
#endif 
#if GCS_OUTPUT == 1
    print_attitude();
#endif
    }



   
    if (radio_status == 1){
      radio_status=2;   // Radio frame read
      ch1_old = ch1;
      ch2_old = ch2;
      ch3_old = ch3;
      ch4_old = ch4;
      ch_aux_old = ch_aux;
      ch_aux2_old = ch_aux2;
      #if SPEKTRUM==1
        ch1 = channel_filter(RxGetChannelPulseWidth(2),ch1_old);         // Aileron
        ch2 = channel_filter(RxGetChannelPulseWidth(3),ch2_old);         // Elevator
        ch3 = channel_filter(RxGetChannelPulseWidth(1),ch3_old);         // Throttle
        ch4 = channel_filter(RxGetChannelPulseWidth(4),ch4_old);         // Rudder
        ch_aux = channel_filter(RxGetChannelPulseWidth(5),ch_aux_old);   // Aux
        ch_aux2 = channel_filter(RxGetChannelPulseWidth(6),ch_aux2_old); // Aux2
      #else // Read radio channel values
        ch1 = RxGetChannelPulseWidth(1);     // Aileron
        ch2 = RxGetChannelPulseWidth(2);     // Elevator
        ch3 = RxGetChannelPulseWidth(3);     // Throttle
        ch4 = RxGetChannelPulseWidth(4);     // Rudder
        ch_aux = RxGetChannelPulseWidth(5);  // Aux
        ch_aux2 = RxGetChannelPulseWidth(6); // Aux2
      #endif      
      
      // Commands from radio Rx... 
      // Stick position defines the desired angle in roll, pitch and yaw
      Stick_cmd_roll_old = Stick_cmd_roll;
      Stick_cmd_roll = (ch1-Neutral[1])/13.0;
      Stick_cmd_roll_diff = Stick_cmd_roll-Stick_cmd_roll_old;

     if ((ch2 >(Neutral[2]+DEAD_CENTER)) || (ch2 < (Neutral[2]-DEAD_CENTER)))
            aux_float2 = (Neutral[2]-ch2)/150.0;         // pitch hold on elevator
     else aux_float2=0;
      
      Stick_cmd_pitch +=aux_float2;
      Stick_cmd_pitch_diff = aux_float2;

      Stick_cmd_pitch = constrain(Stick_cmd_pitch, - MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);  // Pitch control limited to +/- MAX_PITCH_ANGLE

/*   
      aux_float = (ch4-Neutral[4])/200.0;  // No heading lock on Yaw
      Stick_cmd_yaw +=aux_float;
      Stick_cmd_yaw_diff = aux_float;
 */     
      Stick_cmd_yaw_old = Stick_cmd_yaw;
      Stick_cmd_yaw = (Neutral[4]-ch4)/13.0;
      Stick_cmd_yaw_diff = Stick_cmd_yaw-Stick_cmd_yaw_old;
    
     // ***** HEADING LOCK *****
     // when Yaw stick is release at the neutral position -> the current heading is stored as the TargetHeading
      if ((ch4 <(Neutral[4]+DEAD_CENTER)) && (ch4 > (Neutral[4]-DEAD_CENTER)))
         { if (hlock==0)
           {
#if HEADING_LOCK == 1    
               if (hmc5843ok==1) TargetHeading = ToDeg(APM_Compass.Heading);   // Magnetic based heading lock 
               else TargetHeading = ToDeg(yaw);                                // Inertial based heading lock 
#else
               TargetHeading = ToDeg(yaw);                                     // Inertial based heading lock
#endif         
                 yaw_I=0;  // reset the Integral term
                 hlock=1;
              }
          }
      else if ((ch4 >(Neutral[4]+DEAD_CENTER)) || (ch4 < (Neutral[4]-DEAD_CENTER))) 
           { if (hlock==1) hlock=0;
           }
  
      // I use K_aux to adjust gains linked to a knob in the radio... [very usefull to tune the PID gain]
      //K_aux = K_aux*0.8 + ((ch_aux2-1500)/100.0 + 0.6)*0.2;
      K_aux = (15+((ch_aux2-1500)/100.0 + 1))/10;
      if (K_aux < 0) K_aux = 0;
        
         // We read the Switch Mode from Channel 5
      if ((ch_aux > 1800) && (GPS.fix>=1))  // We really need to switch it ON from radio to activate GPS hold
      { AP_mode = 1;           // Position hold mode (GPS position hold) if GPS.fix is OK
        digitalWrite(7,HIGH); // Yellow LED On
      } else
      { AP_mode = 2;          // Normal mode (Stabilization mode)
        digitalWrite(7,LOW); // Yellow LED off
      }
    } else if (radio_status==0)
      {  // Radio_status = 0 Lost radio signal => Descend...
      AP_mode = 2;
      digitalWrite(7,HIGH);  // Yellow LED 
      digitalWrite(5,HIGH);  // Red Led
      ch3--;   // Descend  (Reduce throttle)
      if (ch3<MIN_THROTTLE)
        ch3 = MIN_THROTTLE;
      Stick_cmd_roll = 0;     // Stabilize to roll=0
      Stick_cmd_pitch = 0;
      Stick_cmd_yaw = 0;     
      
      Attitude_control_stable();  // PID algorithm is running here
      
      call_servomix();  // Call the main servos mixer output
      
      }  
#ifdef IsGPS        
      if (AP_mode==1)  // GPS Position Hold
    {
      if (target_position==0)   // If this is the first time we switch to Position control, actual position is our target position
      {
        target_lattitude = GPS.latitude;
        target_longitude = GPS.longitude;
        target_position=1;
        //target_sonar_altitude = sonar_value;
        //Initial_Throttle = ch3;
        // Reset I terms
        altitude_I = 0;
        gps_roll_I = 0;
        gps_pitch_I = 0;
      }        
    }
    else
      target_position=0;

    //Read GPS
     GPS.update();
     if(GPS.fix>=1) 
     { 
        if (GPS.new_data)  // New GPS data?
      {
        GPS_timer_old=GPS_timer;   // Update GPS timer
        GPS_timer = millis();
        GPS_Dt = (GPS_timer-GPS_timer_old)*0.001;   // GPS_Dt
        GPS.new_data=0;  // We Reset the flag...
        
	COGX = cos(ToRad(GPS.ground_course/100.0));  //Course overground X axis
	COGY = sin(ToRad(GPS.ground_course/100.0));  //Course overground Y axis

#if DEBUG_SUBSYSTEM == 0
      //Output GPS data
        Serial.print("GPS -> GPS_Dt:");         Serial.print(GPS_Dt);
        Serial.print("    lat:");               Serial.print(GPS.latitude);
        Serial.print("    lon:");               Serial.print(GPS.longitude);
        Serial.print("    alt:");               Serial.print(GPS.altitude/100.0);
        Serial.print("    spd:");               Serial.print(GPS.ground_speed/100.0);
        Serial.print("    crs:");               Serial.print(GPS.ground_course/100.0);
        Serial.println();
#endif        
        if (GPS.fix)
          digitalWrite(6,HIGH);  // GPS Fix => Blue LED
        else
          digitalWrite(6,LOW);

        if (AP_mode==1)
        { if ((target_position==1) && (GPS.fix))
          { Position_control(target_lattitude,target_longitude);  // Call position hold routine
          }
        else
          { Serial.println("GPS: NOFIX");
            command_gps_roll=0;
            command_gps_pitch=0;
          }
        }
      }
     }
#endif 
    // Attitude control 

    Attitude_control_stable();    // PID algorithm is running here    
     
    if (ch3 > (MIN_THROTTLE+40))  // Minimun throttle to start control
      {
        call_servomix();  // Call the main servos mixer output
      }
    else
      { // If throttle command is low, motor are stopped
      roll_I  = 0;  // reset I terms...
  //    pitch_I = 0;
  //    yaw_I   = 0; 
  //    Stick_cmd_pitch_diff=0;
  //    Stick_cmd_yaw_diff=0;
      control_roll=0;    // stabilisation stopped, allow manual test of yaw, pitch servos mixer

      Servo_Timer2_set(0,MIN_THROTTLE);                // ESC Right motor
      Servo_Timer2_set(1,MIN_THROTTLE);                // ESC Left motor

      /* direct control no stab - test debug only
      Servo_Timer2_set(2,ch2 + (ch4 - Neutral[4]));    // Servo Right motor
      Servo_Timer2_set(3,ch4 - (ch2 - Neutral[2]));    // Servo Left motor
      */
  //    Servo_Timer2_set(2,ch4  - control_pitch - control_yaw);    // Servo Right motor
  //    Servo_Timer2_set(3,ch4  + control_pitch - control_yaw);    // Servo Left motor
  
      Servo_Timer2_set(2,ch2 + (ch4 - Neutral[4])  - control_pitch - control_yaw);    // Servo Right motor
      Servo_Timer2_set(3,ch4 - (ch2 - Neutral[2])  + control_pitch - control_yaw);    // Servo Left motor

     
      // Initialize yaw command to actual yaw for heading lock
#if HEADING_LOCK == 1    
      if (hmc5843ok==1) TargetHeading = ToDeg(APM_Compass.Heading);   // Magnetic based heading lock 
      else TargetHeading = ToDeg(yaw);                                // Inertial based heading lock 
#else
      TargetHeading = ToDeg(yaw);                                     // Inertial based heading lock
#endif              
      Stick_cmd_yaw = 0;
      Stick_cmd_yaw_diff = 0;
      hlock=0;
      }   
      
#if DEBUG_SUBSYSTEM == 1
                                // display de channel Rx ouputs
    Serial.print("\t\tCh1:");         Serial.print(ch1);
    Serial.print(",Ch2:");            Serial.print(ch2);
    Serial.print(",Ch3:");            Serial.print(ch3);
    Serial.print(",Ch4:");            Serial.print(ch4);
    Serial.print(",Aux:");            Serial.print(ch_aux);
    Serial.print(",Ax2:");            Serial.print(ch_aux2);
    Serial.print(",K_aux:");          Serial.print(K_aux);
                                // display the output for the servos
    Serial.print("\t\tESC1:");         Serial.print(chESC1);
    Serial.print("\tESC2:");           Serial.print(chESC2);
    Serial.print("\tSRV1:");           Serial.print(chSERVO1);
    Serial.print("\tSRV2:");           Serial.println(chSERVO2);            

#endif    
#if DEBUG_SUBSYSTEM == 2        // debug the Rx stick command and the PID error
    Serial.print("cmd_roll:");        Serial.print(Stick_cmd_roll);
    Serial.print(",cmd_pitch:");      Serial.print(Stick_cmd_pitch);
    Serial.print(",cmd_yaw:");        Serial.print(Stick_cmd_yaw);
    Serial.print("\terr_roll:");      Serial.print(err_roll);
    Serial.print(",err_pitch:");      Serial.print(err_pitch);
    Serial.print(",err_yaw:");        Serial.print(err_yaw);
    Serial.print(",K_aux:");          Serial.print(K_aux);
    Serial.print(",MAG_Heading:");    Serial.println(ToDeg(APM_Compass.Heading));
#endif 
#if DEBUG_SUBSYSTEM == 3         // debug the GPS
// GPS Properties
//	time;			///< GPS time in milliseconds from the start of the week
//	latitude;		///< latitude in degrees * 10,000,000
//	longitude;		///< longitude in degrees * 10,000,000
//	altitude;		///< altitude in cm
//	ground_speed;	        ///< ground speed in cm/sec
//	ground_course;	        ///< ground course in 100ths of a degree
//	speed_3d;		///< 3D speed in cm/sec (not always available)
//      num_sats;		///< Number of visible satelites
//	fix;			///< true if we have a position fix
    Serial.print("GPS:");
    Serial.print(" Time:");           Serial.print(GPS.time);
    Serial.print(" Fix:");            Serial.print((int)GPS.fix);
    Serial.print(" Lat:");            Serial.print(GPS.latitude);
    Serial.print(" Lon:");            Serial.print(GPS.longitude);
    Serial.print(" Alt:");            Serial.print(GPS.altitude/100.0);
    Serial.print(" Speed:");          Serial.print(GPS.ground_speed/100.0);
    Serial.print(" Course:");         Serial.print(GPS.ground_course/100.0);
    Serial.println();
#endif    
#if DEBUG_SUBSYSTEM == 4     // debug the IMU sensors
    Serial.print("AN[0]:");                Serial.print(AN[0]);
    Serial.print(", AN[1]:");              Serial.print(AN[1]);
    Serial.print(", AN[2]:");              Serial.print(AN[2]);
    Serial.print(", AN[3]:");              Serial.print(AN[3]);
    Serial.print(", AN[4]:");              Serial.print(AN[4]);
    Serial.print(", AN[5]:");              Serial.println(AN[4]);
    
    Serial.print("gyro_offset_roll:");     Serial.print(AN_OFFSET[0]);
    Serial.print(", gyro_offset_pitch:");  Serial.print(AN_OFFSET[1]);
    Serial.print(", gyro_offset_yaw:");    Serial.print(AN_OFFSET[2]);
    Serial.print(", acc_offset_x:");       Serial.print(AN_OFFSET[3]);
    Serial.print(", acc_offset_y:");       Serial.print(AN_OFFSET[4]);
    Serial.print(", acc_offset_z:");       Serial.println(AN_OFFSET[5]);      
#endif    
    }
}

void read_AHRS(void)
{
	// Get gyro and accel data and perform IMU calculations
	//-----------------------------------------------------

    Read_adc_raw();			// Get current values for IMU sensors	
	Matrix_update(); 		// Integrate the DCM matrix
	Normalize();			// Normalize the DCM matrix
	Drift_correction();		// Perform drift correction
	Euler_angles();			// Calculate pitch, roll, yaw for stabilization and navigation
}

///////////////////////////// THE MAIN SERVOS MIXER IS HERE /////////////////////////////////////////////

void call_servomix(void) // Here the main servos mixer 
{
         // OAT type mixer
         // MIXER1: 100%(Ch Throttle) 23%( Ch Ailerons)
         // MIXER2: 100%(Ch Pitch) 100%( Ch Yaw)
         
      // OAT MIXER1 (ch2:throttle, ch1: roll) // OK Working AND tested in flight     
      chESC1  = ch3 + int( ((float)ch1 - Neutral[1]) * 0.23) + int( (float)control_roll * 0.23);  // ESC Right motor 
      chESC2  = ch3 - int( ((float)ch1 - Neutral[1]) * 0.23) - int( (float)control_roll * 0.23);  // ESC Left motor 

      // OAT MIXER2 (ch2:pitch, ch4: yaw) // OK Working AND tested in flight 

 //     chSERVO1  =  ch4  - control_pitch - control_yaw;  // Right Servo  
 //     chSERVO2  =  ch4  + control_pitch - control_yaw;  // Left Servo  
      
      chSERVO1  =  ch2 + (ch4 - Neutral[4]) - control_pitch - control_yaw;  // Right Servo  (ch2:pitch, ch4: yaw)
      chSERVO2  =  ch4 - (ch2 - Neutral[2]) + control_pitch - control_yaw;  // Left Servo  

// Direct control : no stab for test and debug only
//      chSERVO1  =  ch2 + (ch4 - Neutral[4]);  // Right Servo  (ch2:pitch, ch4: yaw) 
//      chSERVO2  =  ch4 - (ch2 - Neutral[2]);  // Left Servo  
      
      // Course limit of the servo      
      chESC1 = constrain(chESC1, 1100, 1900);
      chESC2 = constrain(chESC2, 1100, 1900);
      chSERVO1 = constrain(chSERVO1, 900, 2100);
      chSERVO2 = constrain(chSERVO2, 900, 2100);
      
      Servo_Timer2_set(0,chESC1);        // ESC Right motor
      Servo_Timer2_set(1,chESC2);        // ESC Left motor
      Servo_Timer2_set(2,chSERVO1);      // Servo Right motor
      Servo_Timer2_set(3,chSERVO2);      // Servo Left motor
}
