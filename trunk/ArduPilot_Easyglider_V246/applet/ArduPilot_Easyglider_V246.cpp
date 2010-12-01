/*By Chris Anderson & Jordi Munoz*/
/* ArduPilot Version 2.4.6 */
//
/* Updated version by Jean-Louis Naudin */
//
//  Last Update : 11-13-09
//
// 11-01-09 JLN : Added starting_wp = 1
// 11-02-09 JLN : Added GPS: gpsfix status for telemetry (System tab )
// 11-02-09 JLN : Added WPT: current_wp for telemetry (System tab )
// 11-04-09 JLN : Added HappyKillmore code for GPS Emulator (GPS_EMUL tab)
// 11-04-09 JLN : added Get_Home_Position(void) -> stored into : home_lon, home_lat, home_alt (see Waypoint tab)
// 11-04-09 JLN : Added A-1 GPS_PROTOCOL 3 in the easyglider24.h header file for GPS Emulator
// 11-05-09 JLN : added Save_Home_Position(void) -> to store the virtual home postion and alt for the GPS Emulator (see System tab)
// 11-08-09 JLN : correction of the air_speed_offset bug which gives -289 (!!!) of airspeed...( Sensors tab )
// 11-09-09 JLN : Air_speed_offset bug now corrected, added zasp_req=1 a request flag for Zero Air Speed bias during the catch_analogs Mux
// 11-11-09 JLN : Added the self-learning mode !!!
// 11-13-09 JLN : Updated the Record_New_Waypoint() function, automatic wplist reset for the first wp
// 11-13-09 JLN : Added 8-10 GSP_MINIMUM, the desired minimum ground speed in case of strong front wind

/* Special thanks to: 
  -Bill Premerlani
  -HappyKillMore
  -James Cohen.
  -JB from rotorFX.
  -Automatik.
  -You Name Here.
  -Fefenin
  -Peter Meister
  -Remzibi
  If i forgot a name please tell me!!*/
/*
Todo:
-Add cross-track error. 
-Improve AirSpeed hold and make it in function with GroundSpeed.
 */
 
#include <avr/eeprom.h>
#include <avr/io.h>
#include "Aeasyglider24.h" //Loading the airframe settings. The Header file must bu located in the same directory than the project

//GPS SIRF configuration strings... 
#define SIRF_BAUD_RATE_4800    "$PSRF100,1,4800,8,1,0*0E\r\n"
#define SIRF_BAUD_RATE_9600    "$PSRF100,1,9600,8,1,0*0D\r\n"
#define SIRF_BAUD_RATE_19200    "$PSRF100,1,19200,8,1,0*38\r\n"
#define SIRF_BAUD_RATE_38400    "$PSRF100,1,38400,8,1,0*3D\r\n"  
#define SIRF_BAUD_RATE_57600    "$PSRF100,1,57600,8,1,0*36\r\n"
#define GSA_ON   "$PSRF103,2,0,1,1*27\r\n"   // enable GSA
#define GSA_OFF  "$PSRF103,2,0,0,1*26\r\n"   // disable GSA
#define GSV_ON   "$PSRF103,3,0,1,1*26\r\n"  // enable GSV
#define GSV_OFF  "$PSRF103,3,0,0,1*27\r\n"  // disable GSV
#define USE_WAAS   1     //1 = Enable, 0 = Disable, good in USA, slower FIX... 
#define WAAS_ON    "$PSRF151,1*3F\r\n"       // enable WAAS
#define WAAS_OFF   "$PSRF151,0*3E\r\n"       // disable WAAS

//GPS Locosys configuration strings...
#define USE_SBAS 0
#define SBAS_ON "$PMTK313,1*2E\r\n"
#define SBAS_OFF "$PMTK313,0*2F\r\n"

#define NMEA_OUTPUT_5HZ "$PMTK314,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 5HZ  
#define NMEA_OUTPUT_4HZ "$PMTK314,0,4,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 4HZ 
#define NMEA_OUTPUT_3HZ "$PMTK314,0,3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 3HZ 
#define NMEA_OUTPUT_2HZ "$PMTK314,0,2,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 2HZ 
#define NMEA_OUTPUT_1HZ "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GGA and RMC to 1HZ

#define LOCOSYS_REFRESH_RATE_200 "$PMTK220,200*2C" //200 milliseconds 
#define LOCOSYS_REFRESH_RATE_250 "$PMTK220,250*29\r\n" //250 milliseconds

#define LOCOSYS_BAUD_RATE_4800 "$PMTK251,4800*14\r\n"
#define LOCOSYS_BAUD_RATE_9600 "$PMTK251,9600*17\r\n"
#define LOCOSYS_BAUD_RATE_19200 "$PMTK251,19200*22\r\n"
#define LOCOSYS_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define Battery_Voltage(x) (x*(INTPUT_VOLTAGE/1024.0))/(100000.0/(240000.0+100000.0))/1000.0 //I explained this in the Voltage Divider Theory! 
#define Pressure_MTS(x) sqrt((((x*(INTPUT_VOLTAGE/1024.0))/1.0)*2.0)/1.225) //Converts to AirSpeed in meters for second, i will later explain this. 
#define Pressure_KMH(x) Pressure_MTS(x)*3.6 //Kilometers per hour, never use English system please. This stuff is international. 

#define PID_dt 20 //Servo refresh time in milliseconds 
#define Start_Byte 0x0E //Used to read the EEPROM 


/***************************************************************************
 General variables
 **************************************************************************/
 
#include "WProgram.h"
void setup();
void loop();
void navigation(void);
void stabilization(void);
int calc_roll(float error,float dt);
int altitude_error(int PID_set_Point, int PID_current_Point);
int heading_error(int PID_set_Point, int PID_current_Point);
byte PID_throttle(int PID_set_Point, int PID_current_Point, int dt);
int PID_roll(int PID_error, byte absolute, int dt);
int PID_error(int PID_set_Point, int PID_current_Point);
int PID_pitch(int PID_error, int dt);
void init_gps(void);
void fast_init_gps(void);
void decode_gps(void);
void GPS_join_data(void);
void configure_gps(void);
void change_to_sirf_protocol(void);
void Wait_GPS_Fix(void);
float to_float_7(long value);
float to_float_6(long value);
void init_gps(void);
void fast_init_gps(void);
void decode_gps(void);
void Save_Home_Position(void);
void Get_Home_Position(void);
void print_remzibi_debug(void);
void print_data_emulator(void);
void init_gps(void);
void fast_init_gps(void);
void decode_gps(void);
void init_gps(void);
void fast_init_gps(void);
void decode_gps(void);
void checksum(byte data);
void wait_for_data(byte many);
long Get_Relative_Altitude();
unsigned int Hold_Current_Altitude(void);
void update_distance(void);
int calc_bearing(float flat1, float flon1, float flat2, float flon2);
unsigned int calc_dist(float flat1, float flon1, float flat2, float flon2);
void Analog_Init(void);
int Anal_Read(uint8_t pin);
void Analog_Reference(uint8_t mode);
void catch_analogs(void);
int airSpeed(void);
int y_axis(void);
int x_axis(void);
int get_roll(void);
int get_pitch(void);
void sensor_z(void);
void Init_servos(void);
void pulse_servos(int roll, int pitch);
void pulse_servo_0(long porcent);
void pulse_servo_1(long angle);
void pulse_servo_2(long angle);
void test_servos(void);
int read_Ch1(void);
int read_Ch2(void);
void init_ardupilot(void);
void Load_Settings(void);
void Save_Launch_Parameters_Flagged(void);
void Restore_Launch_Parameters(void);
byte Tx_Switch_Status(void);
float reset(float value);
void maxis(void);
void print_data(void);
void request_cmd(void);
void Record_New_Waypoint(void);
void Waypoint_Switcher(int distance, byte radius);
void Set_New_Waypoint(int current_waypoint);
void print_waypoints(void);
void test_throttle(void);
void calibrate_servos(void);
void Print_EEPROM(void);
const float kp[]={
  roll_P,pitch_P}
,ki[]={
  roll_I,pitch_I}; //PI gains


union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

/*Flight GPS variables*/
int gpsFix=1; //This variable store the status of the GPS
float lat=0; // store the Latitude from the gps
float lon=0;// Store guess what?
float alt_MSL=0; //This is the alt.
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=0;//This is the runaway direction of you "plane" in degrees
float climb_rate=0; //This is the velocity you plane will impact the ground (in case of being negative) in meters for seconds
char data_update_event=0; 

unsigned long PID_timer=0;
unsigned long NAV_timer=0;

//
uint8_t MuxSel=0;
uint8_t analog_reference = DEFAULT;
int16_t analog_buffer[6];
volatile uint8_t ADC_flag=0;
//
float analog0=511; //Roll Analog  
float analog1=511; //Pitch Analog
float analog2=511; //Z Analog
float analog3=511; //I have to change this name is the airspeed sensor... SOON! 
float analog5=511;

int max_ir=0;
int air_speed_offset=0;
int air_speed_hold=0;
unsigned int zasp_req=0; 

byte throttle_set_point=0;
int roll_set_point=0; //Stores the desired set point of roll and pitch
int pitch_set_point=0;
unsigned int hold_Alt=0; 
unsigned int rx_Ch[2];

unsigned int wp_distance=0; 
unsigned int wp_bearing=0;
unsigned int wplist_flag=0;
unsigned int wpreset_flag=0;
//byte wp_Mode;

byte options=0; 
char roll_trim= -4; //You know like the transmitter trims 
char pitch_trim= 10; 
int max_alt=0;
int max_spd=0;
byte wp_number=3; //Number of waypoints defined.... Go to to Waypoint tab...
byte current_wp=0;//Store the current waypoint...
byte wp_radius=20; //Radius of the waypoint, normally set as 20 meters.
unsigned int launch_alt=0; //Launch 

//home position parameters. 
float home_lat=0;
float home_lon=0;
unsigned int home_alt=0; 
byte starting_wp=1;//Store the starting waypoint...

//Stores the actual waypoint we are trying to reach. 
float wp_current_lat=0;
float wp_current_lon=0;
unsigned int wp_current_alt=0; 
int last_waypoint=999;

long refresh_rate=0;
const float t7=1000000.0;

float Batt_Volt =0;
/***************************************************************************
 NMEA variables
 **************************************************************************/
#if GPS_PROTOCOL == 0 // This condition is used by the compiler only....
/*GPS Pointers*/
char *token; //Some pointers
char *search = ",";
char *brkb, *pEnd;
char gps_buffer[200]; //The traditional buffer.
#endif
/***************************************************************************
GPS EMULATOR variables
 **************************************************************************/
#if GPS_PROTOCOL == 3 // This condition is used by the compiler only....
/*GPS Pointers*/
char *token; //Some pointers
char *search = ",";
char *brkb, *pEnd;
char gps_buffer[200]; //The traditional buffer.
char gps_GGA[80]; // added for GPS emulator
char gps_RMC[80]; // added for GPS emulator
//virtual home position parameters. 
float vhome_lat=48.56464;
float vhome_lon=2.87097;
unsigned int vhome_alt=20;
#endif
/***************************************************************************
 SIRF variables
 **************************************************************************/
#if GPS_PROTOCOL == 1
 //GPS stuff please read SiRF-Binary-Protocol-Reference-Manual page 87 for more information
 byte gps_buffer[90]={ 0x24,0x50,0x53,0x52,0x46,0x31,0x30,0x30,0x2C,0x30,0x2C,
  0x35,0x37,0x36,0x30,0x30,0x2C,0x38,0x2C,0x31,0x2C,0x30,
  0x2A,0x33,0x37,0x0D,0x0A}; //Buffer that store all the payload coming from the GPS
 
 const byte gps_ender[]={0xB0,0xB3};  //Used to configure Sirf GPS
#endif
/***************************************************************************
 uBlox Variables
 **************************************************************************/
#if GPS_PROTOCOL == 2
//uBlox Checksum
byte ck_a=0;
byte ck_b=0;
long iTOW=0; //GPS Millisecond Time of Week
long alt=0; //Height above Ellipsoid 
long speed_3d=0; //Speed (3-D)  (not used)
#endif


/***************************************************************************

 **************************************************************************/
void setup()
{
  init_ardupilot();  //Initialize ardupilot...
  current_wp=starting_wp;
  zasp_req=1; // zero air speed request flag
}
void loop()//Main Loop
{
  navigation();//Calculate all the setpoints for the roll, pitch and throttle. Function localted in "Amain" tab.
  stabilization();//Control of all the actuators using IRs and Pressure Sensor... and the desired Setpoints for each one. Function localted in "Amain" tab.
  catch_analogs(); //Reads and average the sensors when is doing nothing... Function localted in "Sensors" tab.
  decode_gps();  //Reads and average the GPS when is doing nothing...
  request_cmd(); // Exec the requested commands for the Self-Learning Mode... Function located in "System" tab

  #if GPS_PROTOCOL == 3
  // print_remzibi(); //Function localted in "System" tab.
   print_data_emulator();  // Required for GPS emulator
  #else
   print_data(); //Function localted in "System" tab. Set to normal flight
  #endif
  
  if(gpsFix != 0x00)//No valid GPS
  {
  read_Ch1(); //reading the receiver with very short delays... Function localted in "Servos" tab.
  read_Ch2(); //Reading the receiver with very short delays... Function localted in "Servos" tab.
  }
}


//v2.4.6
/****************************************************************
 * navigation(), this part will calculate the setpoints of the PID that control the servos... 
 ****************************************************************/
void navigation(void)
{
  if((data_update_event&0x01)==0x01) //Verify if we have new GPS data.. Is like a flag... 
  {
    data_update_event&=0xFE; //Resetting flag, 0xFE is equeal to ~0x01 (not) 

    float dt_t = (float)(millis()-NAV_timer)/1000.0; //Timer

    if (Tx_Switch_Status()==0x01) //Check if we are in mode 1 (transmitter switch in middle position) if yes enter to waypoint mode
    {
      Set_New_Waypoint(current_wp); //Loading current waypoint.
      update_distance(); //Updating the distance between the current position and the destination waypoints
      Waypoint_Switcher(wp_distance, wp_radius); //Waypoint switcher... 
    }
    else if (Tx_Switch_Status()==0x02) //If we are not in mode 2 please return home (waypoint 0)
    {
      //Here we put home position. 
      Set_New_Waypoint(0); //Waypoint zero is home.
      update_distance(); //This one will activate the nuclear missile and update the distance. 
    }

    wp_bearing=calc_bearing(lat, lon, wp_current_lat, wp_current_lon); //Calculating bearing

    //Now calculates the roll set point, in order to turn to the desired heading.. 
#if FAKE_BEARING == 1
    roll_set_point= calc_roll(heading_error((int)DESIRED_FAKE_BEARING, ground_course),dt_t); //Faking the bearing.
#endif
#if FAKE_BEARING == 0
    roll_set_point= calc_roll(heading_error((int)wp_bearing, ground_course),dt_t); //Calculate the roll set point depending the heading error. 
#endif
    NAV_timer=millis(); //Resetting timer    
  }

  if((data_update_event&0x02)==0x02)///Checking new GPS data flag
  {
    int alt_error = 0;
    data_update_event&=0xFD; //Clearing flag 
    
    alt_error =altitude_error(Hold_Current_Altitude(), Get_Relative_Altitude()); //Holding the altitude before switch to RTL. 

    //Here will take the altitude error to control the throttle, has proportional and limits.
    air_speed_hold=constrain(AIRSPEED_CENTRAL+(alt_error*ALTITUDE_ERROR_AIRSPEED_PROPORTIONAL),ALTITUDE_ERROR_AIRSPEED_MIN,ALTITUDE_ERROR_AIRSPEED_MAX); 
    
    if(ground_speed<=GSP_MINIMUM) air_speed_hold=air_speed_hold+GSP_MINIMUM;

    //Here will take the altitude error to move the elevator, has proportional and limits
    pitch_set_point=constrain(alt_error*ALTITUDE_ERROR_PITCH_PROPORTIONAL,ALTITUDE_ERROR_PITCH_MIN,ALTITUDE_ERROR_PITCH_MAX);

    maxis(); //Storing max altitude and speed. 
  }
}

/****************************************************************
 * stabilization(), this part will control the aircraft! I guess is the most important part... 
 ****************************************************************/
void stabilization(void)
{
  unsigned int t_dt=millis()-PID_timer; //Timer... 

  int Pitch=0;
  int Roll=0;

  if(t_dt > PID_dt)
  {
    //If valid gps data, navigate, else just stabilize... 
    if(gpsFix == 0x00) 
    {
      //Calculating Roll
      Roll=PID_roll(PID_error(roll_set_point, (roll_trim+get_roll())*WALK_AROUND),roll_abs,t_dt); 
      //Calculating Pitch
      Pitch=PID_pitch(PID_error(pitch_set_point+((abs(get_roll())*PITCH_COMP)), pitch_trim+get_pitch()),t_dt);
      //Sending the values to the servos. 
      pulse_servos(Roll,Pitch);
    }
    else //This is fly by wire
    {
      if((abs(read_Ch1())>25) || (abs(read_Ch2())>25))//Fly by wire, check if we move the sticks 
      {
        Roll=PID_roll(PID_error(((float)read_Ch1()*FLY_BY_WIRE_GAIN_ROLL), roll_trim+get_roll()),roll_abs,t_dt);
        Pitch=PID_pitch(PID_error(((float)read_Ch2()*FLY_BY_WIRE_GAIN_PITCH), pitch_trim+get_pitch()),t_dt);
        pulse_servos(Roll,Pitch);
        air_speed_hold=FLY_BY_WIRE_SPEED_SETPOINT;
      }
      else //This is Just stabilize, no navigation. 
      {
        Roll=PID_roll(PID_error(roll_trim, roll_trim+get_roll()),roll_abs,t_dt);
        Pitch=PID_pitch(PID_error(pitch_trim, pitch_trim+get_pitch()),t_dt);
        pulse_servos(Roll,Pitch);//Stabilization only 
        air_speed_hold=GPS_ERROR_SPEED_SETPOINT;
      }      
    }

    //Throttle
    throttle_set_point=PID_throttle(air_speed_hold, airSpeed(), t_dt);
    pulse_servo_0((throttle_set_point));
    //Refreshing the Z sensor.

    PID_timer=millis(); //resetting timer... 
  } 
}
//V2.4.4
/****************************************************************
 * This function calculate the desired roll angle... 
 ****************************************************************/
int calc_roll(float error,float dt)
{
  static float I;
  static float D;
  static float previous_error;
  
  //Integratior part
  I+= (float)error*dt; //1000 microseconds / 1000 = 1 millisecond
  I= constrain(I,-10,10); //Limits
  I=reset(I);
  error=(error*head_P)+(I*head_I);
  
  //Derivation part
  D=(error-previous_error)/dt;
  previous_error=error;
  
  error=(error*head_P)+(I*head_I)+(D*head_D);
  
  return constrain(error,head_error_min,head_error_max); //Limiting the roll.... 
}

/****************************************************************
 * Altitude hold error... 
 ****************************************************************/
int altitude_error(int PID_set_Point, int PID_current_Point)
{
  int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  return constrain((PID_error),ALTITUDE_ERROR_MIN,ALTITUDE_ERROR_MAX); 
}

/***************************************************************************
 * //Computes heading error, and choose the shortest way to reach the desired heading.
 ***************************************************************************/
int heading_error(int PID_set_Point, int PID_current_Point)
{
 float PID_error = PID_set_Point - PID_current_Point;

 if (PID_error > 180) {
   PID_error -= 360;
 }

 if (PID_error < -180) {
   PID_error += 360;
 }

 return PID_error;
}
/*****************************************
 * Proportional Integrator Control for Throttle
 *****************************************/
byte PID_throttle(int PID_set_Point, int PID_current_Point, int dt)
{
  //int roll_previous_error=0;
  static int throttle_output;
  static float old_throttle_output;
  static float throttle_Integrator;
  //float roll_D=0;

  int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  
  throttle_output= (throttle_kp*PID_error);//Adding proportional

  throttle_Integrator+= (float)(((float)PID_error*((float)dt/1000.0))*(float)throttle_ki); 
  throttle_Integrator=constrain(throttle_Integrator,0,throttle_Integrator_max);
  throttle_Integrator=reset(throttle_Integrator);
  throttle_output+=throttle_Integrator;
  throttle_output+=((float)PID_set_Point*(float)throttle_absolute);

  //Plus all the PID results and limit the output... 
  throttle_output = constrain(throttle_output,0,throttle_max); //
  
  old_throttle_output = (float)(((float)old_throttle_output*.90) + ((float)throttle_output*.10)); 
  
  return old_throttle_output; //Returns the result       
}

/*****************************************
 * Proportional Integrator Control for roll
 *****************************************/
int PID_roll(int PID_error, byte absolute, int dt)
{

  //int roll_previous_error=0;
  static int roll_output;
  static float roll_Integrator;
  //float roll_D=0;

  //int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  roll_output= (kp[0]*PID_error);//Adding proportional

  roll_Integrator+= (float)(((float)PID_error*((float)dt/1000.0))*(float)ki[0]); 
  roll_Integrator=constrain(roll_Integrator,roll_Integrator_min,roll_Integrator_max);
  roll_Integrator=reset(roll_Integrator);
  roll_output+=roll_Integrator;

  //roll_output+=((float)PID_set_Point*(float)absolute);

  //Plus all the PID results and limit the output... 
  roll_output = constrain(roll_output,roll_min,roll_max); //

  return roll_output; //Returns the result       
}

/*****************************************
 * PID ERROR
 *****************************************/
 int PID_error(int PID_set_Point, int PID_current_Point)
{
 
  int PID_error=PID_set_Point-PID_current_Point;
 
 return  PID_error;
}
/*****************************************
 * Proportional Integrator Control for pitch
 *****************************************/
int PID_pitch(int PID_error, int dt)
{ 
  
  static int pitch_output;
  static float pitch_Integrator;
  //static int pitch_previous_error=0;
  //static float pitch_D=0;  
  //int PID_error=PID_set_Point-PID_current_Point; //Computes the error

  pitch_output= (float)((float)kp[1]*(float)PID_error);//Adding proportional

  pitch_Integrator+= (float)(((float)PID_error*((float)dt/1000.0))*(float)ki[1]); 
  pitch_Integrator=constrain(pitch_Integrator,pitch_Integrator_min,pitch_Integrator_max);
  pitch_Integrator=reset(pitch_Integrator);
  pitch_output+=pitch_Integrator;

  pitch_output = constrain(pitch_output,pitch_min,pitch_max); //PID_P+PID_D

  return pitch_output;  
}

#if GPS_PROTOCOL == 1

/****************************************************************
 Parsing stuff for SIRF binary protocol. 
 ****************************************************************/
void init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  delay(500);
  change_to_sirf_protocol();
  Serial.begin(57600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(500);//Waits fot the GPS to start_UP
  configure_gps();//Function to configure GPS, to output only the desired msg's
  Wait_GPS_Fix();
}
/****************************************************************
 ****************************************************************/
void fast_init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(57600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(100);//Waits fot the GPS to start_UP
  configure_gps();//Function to configure GPS, to output only the desired msg's
}
/****************************************************************
 ****************************************************************/
void decode_gps(void)
{
  static unsigned long GPS_timer=0;
  static byte gps_counter=0; //Another gps counter for the buffer
  static byte GPS_step=0;
  static byte gps_ok=0;//Counter to verify the reciving info
  const byte read_gps_header[]={
    0xA0,0xA2,0x00,0x5B,0x29      };//Used to verify the payload msg header

  if(millis()-GPS_timer > 200) //Timer to execute the loop every 200 ms only.
  {
    if(Serial.available()>0)//Ok, let me see, the buffer is empty?
    {
      switch(GPS_step) //Let see in which step i'am.
      {
      case 0: //This case will verify the header, to know when the payload will begin 
        while(Serial.available()>0)  //Loop if data available
        {
          if(Serial.read()==read_gps_header[gps_ok]) //Checking if the head bytes are equal.. 
          {
            gps_ok++; //If yes increment 1
          }
          else{ 
            gps_ok=0; //Otherwise restart.
          }
          if(gps_ok >= 5) //Ohh 5 bytes are correct, that means jump to the next step, and break the loop
          {
            gps_ok=0;
            GPS_step++;
            break;
          }
        }
        break; 
      case 1: //Will receive all the payload and join the received bytes... 
        while(Serial.available()>0) //Loop if theres something in the buffer 
        {
          gps_buffer[gps_counter++]=Serial.read(); //Read data and store it in the temp buffer

          if(gps_counter>=92) //If we got 92 bytes (all the payload) then... 
          {
            gps_counter=0; //Restart the counter... 
            while(Serial.available()==0){
            }//Wait for the ender bytes 
            if((Serial.read()==gps_ender[0])&&(Serial.read()==gps_ender[1])) //Check if we are in the end of the payload, then... 
            {
              unsigned int gps_checksum_verif=0x29;//Restart the checksum value 
              for(int j=0; j<90; j++)//Checksum verifycation, chachan! this is the moment of the true
              {
                gps_checksum_verif+=gps_buffer[j];//I simply plus all the payload
              }
              if(((gps_buffer[90]<<8)|gps_buffer[91]) == gps_checksum_verif) //Now i will check if the checksum i just made with the received checksum are equal, if yes... 
              {
                GPS_join_data(); //Joing the data
                data_update_event|=0x01;
                data_update_event|=0x02;
                //GPS_print_data(); // and print the values... 
              }
            }
            GPS_step=0; //Restarting.... 
            break;
          }
        }
        break;
      }
      GPS_timer=millis(); //Restarting timer... 
    }
    if(millis() - GPS_timer > 2000){
      digitalWrite(12, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED... 
      gpsFix=1; 
    }
  }
}
/****************************************************************
 ****************************************************************/
void GPS_join_data(void)
{
  gpsFix=(int)gps_buffer[0]<<8; //Here i'm joining the status GPS 2 Bytes to 1 Int. 
  gpsFix|=(int)gps_buffer[1];

  if(gpsFix == 0) //If the status GPS is equals to cero YEAAHH! turn ON the LED
  {
    digitalWrite(12,HIGH);
  }
  else{
    digitalWrite(12,LOW);
  }//If not UHH something is really wrong, don't look at me! is GPS fault!

  byte j=22;//Switching the byte position on the buffer

  //lat = to_float_7(join_4_bytes(gps_buffer[j++], gps_buffer[j++], gps_buffer[j++], gps_buffer[j++]));//Joining latitude Bytes
  longUnion.byte[3] = gps_buffer[j++];
  longUnion.byte[2] = gps_buffer[j++];
  longUnion.byte[1] = gps_buffer[j++];
  longUnion.byte[0] = gps_buffer[j++];
  lat=to_float_7(longUnion.dword);

  //lon = to_float_7(join_4_bytes(gps_buffer[j++], gps_buffer[j++], gps_buffer[j++], gps_buffer[j++]));//Joining longitude Bytes

  longUnion.byte[3] = gps_buffer[j++];
  longUnion.byte[2] = gps_buffer[j++];
  longUnion.byte[1] = gps_buffer[j++];
  longUnion.byte[0] = gps_buffer[j++];
  lon=to_float_7(longUnion.dword);

  j=34;//Switching the byte position on the buffer
  //alt_MSL = (float)(join_4_bytes(gps_buffer[j++], gps_buffer[j++], gps_buffer[j++], gps_buffer[j++]))/100.0;//Joining altitude Bytes
  longUnion.byte[3] = gps_buffer[j++];
  longUnion.byte[2] = gps_buffer[j++];
  longUnion.byte[1] = gps_buffer[j++];
  longUnion.byte[0] = gps_buffer[j++];
  alt_MSL = (float)longUnion.dword/100.0;

  j=39;//Switching the byte position on the buffer
  //ground_speed= (float)join_2_bytes(gps_buffer[j++], gps_buffer[j++])/100.0;//Joining Ground Speed Bytes
  intUnion.byte[1] = gps_buffer[j++];
  intUnion.byte[0] = gps_buffer[j++];
  ground_speed= (float)intUnion.word/100.0;

  if(ground_speed>=.5)//Only updates data if we are really moving... 
  {
    //ground_course=(float)join_2_bytes(gps_buffer[j++], gps_buffer[j++])/100.0;//Joining Course Bytes
    intUnion.byte[1] = gps_buffer[j++];
    intUnion.byte[0] = gps_buffer[j++];
    ground_course= (float)intUnion.word/100.0;
    ground_course=abs(ground_course);//The GPS has a BUG sometimes give you the correct value but negative, weird!! 
  }
  j=45;//Switching the byte position on the buffer
  //climb_rate=(float)join_2_bytes(gps_buffer[j++], gps_buffer[j++])/100.0;  //Joining climb rate Bytes
  intUnion.byte[1] = gps_buffer[j++];
  intUnion.byte[0] = gps_buffer[j++];
  climb_rate= (float)intUnion.word/100.0;

  for(int j=0; j<94; j++)
  {
    gps_buffer[j]=0; //Now is time to say good bye to the buffer..=(.. Every byte was like a son.. 
  } 
}
/****************************************************************
 ****************************************************************/
void configure_gps(void)
{
  const byte gps_header[]={
    0xA0,0xA2,0x00,0x08,0xA6,0x00      };//Used to configure Sirf GPS
  const byte gps_payload[]={
    0x02,0x04,0x07,0x09,0x1B      };//Used to configure Sirf GPS
  const byte gps_checksum[]={
    0xA8,0xAA,0xAD,0xAF,0xC1      };//Used to configure Sirf GPS
  const byte cero=0x00;//Used to configure Sirf GPS

  for(int z=0; z<2; z++)
  { 
    for(int x=0; x<5; x++)//Print all messages to setup GPS
    {
      for(int y=0; y<6; y++)
      {
        Serial.print(byte(gps_header[y]));//Prints the msg header, is the same header for all msg..  
      } 
      Serial.print(byte(gps_payload[x]));//Prints the payload, is not the same for every msg
      for(int y=0; y<6; y++)
      {
        Serial.print(byte(cero)); //Prints 6 ceros
      } 
      Serial.print(byte(gps_checksum[x])); //Print the Checksum
      Serial.print(byte(gps_ender[0]));  //Print the Ender of the string, is same on all msg's. 
      Serial.print(byte(gps_ender[1]));  //ender  
    }
  }  
}

/****************************************************************
 ****************************************************************/

void change_to_sirf_protocol(void)
{
  digitalWrite(13, HIGH);
  Serial.begin(4800); //First try in 4800
  delay(300);
  for (byte x=0; x<=28; x++)
  {
    Serial.print(byte(gps_buffer[x]));//Sending special bytes declared at the beginning 
  }  
  digitalWrite(13, LOW);
  delay(300);
  digitalWrite(13, HIGH);
  Serial.begin(9600); //Then try in 9600 
  delay(300);
  for (byte x=0; x<=28; x++)
  {
    Serial.print(byte(gps_buffer[x]));
  }  
  digitalWrite(13, LOW);  
}  
#endif

/****************************************************************
 ****************************************************************/
void Wait_GPS_Fix(void)//Wait GPS fix...
{
  do
  {
    decode_gps();
    digitalWrite(12,HIGH);
    delay(25);
    digitalWrite(12,LOW);
    delay(25);
  }
  while(gpsFix!=0);//

#if GPS_PROTOCOL == 0 //If NMEA
  do
  {
    decode_gps(); //Reading and parsing GPS data  
    digitalWrite(12,HIGH);
    delay(25);
    digitalWrite(12,LOW);
    delay(25);
  }
  while((data_update_event&0x01!=0x01)&(data_update_event&0x02!=0x02));
#endif
}

/****************************************************************
 ****************************************************************/
float to_float_7(long value) 
{
  return (float)value/(float)10000000;
}
float to_float_6(long value) 
{
  return (float)value/(float)1000000;
}


#if GPS_PROTOCOL == 3
/****************************************************************
 Parsing stuff for NMEA - GPS EMULATOR VERSION
 ****************************************************************/
void init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(9600);
//  Serial.begin(38400);
//  delay(1000);
//  Serial.print(LOCOSYS_BAUD_RATE_38400);
//  Serial.begin(38400);
//  delay(500);
//  Serial.print(LOCOSYS_REFRESH_RATE_250);
//  delay(500);
//  Serial.print(NMEA_OUTPUT_4HZ);
//  delay(500);
//  Serial.print(SBAS_OFF);
 
  delay(1000);
  Serial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(500);
  Serial.println("$PMTK313,1*2e"); 
  delay(500);
  Serial.println("$PMTK301,2*2e"); 
  delay(500);
  Serial.println("$PMTK220,200*2C"); // 5Hz
  delay(500); 
//  Serial.println("$PMTK251,38400*27");
//  delay(500);
  
  
  /* EM406 example init
  Serial.begin(4800); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(1000);
  Serial.print(SIRF_BAUD_RATE_9600);
 
  Serial.begin(9600);
  delay(1000);
  
  Serial.print(GSV_OFF);
  Serial.print(GSA_OFF);
  
  #if USE_WAAS ==1
  Serial.print(WAAS_ON);
  #else
  Serial.print(WAAS_OFF);
  #endif*/
  
  Wait_GPS_Fix();
}
void fast_init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(9600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  //Serial.begin(38400);
}

void decode_gps(void)
{
  const char head_rmc[]="GPRMC"; //GPS NMEA header to look for
  const char head_gga[]="GPGGA"; //GPS NMEA header to look for
  
  static unsigned long GPS_timer=0; //used to turn off the LED if no data is received. 
  
  static byte unlock=1; //some kind of event flag
  static byte checksum=0; //the checksum generated
  static byte checksum_received=0; //Checksum received
  static byte counter=0; //general counter

  //Temporary variables for some tasks, specially used in the GPS parsing part (Look at the NMEA_Parser tab)
  unsigned long temp=0;
  unsigned long temp2=0;
  unsigned long temp3=0;


  while(Serial.available() > 0)
  {
    if(unlock==0)
    {
      gps_buffer[0]=Serial.read();//puts a byte in the buffer

      if(gps_buffer[0]=='$')//Verify if is the preamble $
      {
        counter = 0;
        checksum = 0; 
        unlock=1; 
      }
    }
    /*************************************************/
    else
    {
      gps_buffer[counter]=Serial.read();


      if(gps_buffer[counter]==0x0A)//Looks for \F
      {

        unlock=0;


        if (strncmp (gps_buffer,head_rmc,5) == 0)//looking for rmc head....
        {

          /*Generating and parsing received checksum, */
          for(int x=0; x<100; x++)
          {
            if(gps_buffer[x]=='*')
            { 
              checksum_received=strtol(&gps_buffer[x+1],NULL,16);//Parsing received checksum...
              break; 
            }
            else
            {
              checksum^=gps_buffer[x]; //XOR the received data... 
            }
          }

          if(checksum_received==checksum)//Checking checksum
          {
            /* Token will point to the data between comma "'", returns the data in the order received */
            /*THE GPRMC order is: UTC, UTC status ,Lat, N/S indicator, Lon, E/W indicator, speed, course, date, mode, checksum*/

            strcpy(gps_RMC,gps_buffer);
            token = strtok_r(gps_buffer, search, &brkb); //Contains the header GPRMC, not used

            token = strtok_r(NULL, search, &brkb); //UTC Time, not used
            //time=  atol (token);
            token = strtok_r(NULL, search, &brkb); //Valid UTC data? maybe not used... 


            //Longitude in degrees, decimal minutes. (ej. 4750.1234 degrees decimal minutes = 47.835390 decimal degrees)
            //Where 47 are degrees and 50 the minutes and .1234 the decimals of the minutes.
            //To convert to decimal degrees, devide the minutes by 60 (including decimals), 
            //Example: "50.1234/60=.835390", then add the degrees, ex: "47+.835390=47.835390" decimal degrees
            token = strtok_r(NULL, search, &brkb); //Contains Latitude in degrees decimal minutes... 

            //taking only degrees, and minutes without decimals, 
            //strtol stop parsing till reach the decimal point "."  result example 4750, eliminates .1234
            temp=strtol (token,&pEnd,10);

            //takes only the decimals of the minutes
            //result example 1234. 
            temp2=strtol (pEnd+1,NULL,10);

            //joining degrees, minutes, and the decimals of minute, now without the point...
            //Before was 4750.1234, now the result example is 47501234...
            temp3=(temp*10000)+(temp2);


            //modulo to leave only the decimal minutes, eliminating only the degrees.. 
            //Before was 47501234, the result example is 501234.
            temp3=temp3%1000000;


            //Dividing to obtain only the de degrees, before was 4750 
            //The result example is 47 (4750/100=47)
            temp/=100;

            //Joining everything and converting to float variable... 
            //First i convert the decimal minutes to degrees decimals stored in "temp3", example: 501234/600000= .835390
            //Then i add the degrees stored in "temp" and add the result from the first step, example 47+.835390=47.835390 
            //The result is stored in "lat" variable... 
            lat=temp+((float)temp3/600000);


            token = strtok_r(NULL, search, &brkb); //lat, north or south?
            //If the char is equal to S (south), multiply the result by -1.. 
            if(*token=='S'){
              lat=lat*-1;
            }

            //This the same procedure use in lat, but now for Lon....
            token = strtok_r(NULL, search, &brkb);
            temp=strtol (token,&pEnd,10); 
            temp2=strtol (pEnd+1,NULL,10); 
            temp3=(temp*10000)+(temp2);
            temp3=temp3%1000000; 
            temp/=100;
            lon=temp+((float)temp3/600000);

            token = strtok_r(NULL, search, &brkb); //lon, east or west?
            if(*token=='W'){
              lon=lon*-1;
            }

            token = strtok_r(NULL, search, &brkb); //Speed overground?
            ground_speed= atoi(token);

            token = strtok_r(NULL, search, &brkb); //Course?
            ground_course= atoi(token);

            data_update_event|=0x01; //Update the flag to indicate the new data has arrived. 

            digitalWrite(13,HIGH);
            Serial.print("$");
            Serial.println(gps_RMC);
            delay(10);
            digitalWrite(13,LOW);
            
          }
          checksum=0;
        }//End of the GPRMC parsing

        if (strncmp (gps_buffer,head_gga,5) == 0)//now looking for GPGGA head....
        {
          /*Generating and parsing received checksum, */
          for(int x=0; x<100; x++)
          {
            if(gps_buffer[x]=='*')
            { 
              checksum_received=strtol(&gps_buffer[x+1],NULL,16);//Parsing received checksum...
              break; 
            }
            else
            {
              checksum^=gps_buffer[x]; //XOR the received data... 
            }
          }

          if(checksum_received==checksum)//Checking checksum
          {
            strcpy(gps_GGA,gps_buffer);

            token = strtok_r(gps_buffer, search, &brkb);//GPGGA header, not used anymore
            token = strtok_r(NULL, search, &brkb);//UTC, not used!!
            token = strtok_r(NULL, search, &brkb);//lat, not used!!
            token = strtok_r(NULL, search, &brkb);//north/south, nope...
            token = strtok_r(NULL, search, &brkb);//lon, not used!!
            token = strtok_r(NULL, search, &brkb);//wets/east, nope
            token = strtok_r(NULL, search, &brkb);//Position fix, used!!
            gpsFix =atoi(token); 
            if(gpsFix >=1)
            gpsFix=0;
            else
            gpsFix=1;
            token = strtok_r(NULL, search, &brkb); //sats in use!! Nein...
            token = strtok_r(NULL, search, &brkb);//HDOP, not needed
            token = strtok_r(NULL, search, &brkb);//ALTITUDE, is the only meaning of this string.. in meters of course. 
            alt_MSL=atoi(token);
            if(alt_MSL<0){
              alt_MSL=0;
            }

            if(gpsFix==0x00) digitalWrite(12,HIGH); //Status LED...
            else digitalWrite(12,LOW);
            
            digitalWrite(13,HIGH);  
            Serial.print("$"); 
            Serial.println(gps_GGA);
            delay(10);  
            digitalWrite(13,LOW); 

            data_update_event|=0x02; //Update the flag to indicate the new data has arrived.
          }
          checksum=0; //Restarting the checksum
        }

        for(int a=0; a<=counter; a++)//restarting the buffer
        {
          gps_buffer[a]=0;
        } 
        counter=0; //Restarting the counter
        GPS_timer=millis(); //Restarting timer...
      }
      else
      {
        counter++; //Incrementing counter
        if (counter >= 200)
        {
          //Serial.flush();
          counter = 0; 
          checksum = 0;
          unlock = 0;
        }
      }
    }
  }
  
  if(millis() - GPS_timer > 2000){
      digitalWrite(12, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED... 
      gpsFix=1; 
    }  
}

void Save_Home_Position(void)
{
  launch_alt=vhome_alt;//Storing launch altitude... 
  current_wp=0;

  eeprom_busy_wait(); 
  eeprom_write_byte((byte*)0x0A,starting_wp);

  eeprom_busy_wait(); 
  //eeprom_write_word((unsigned int*)0x01,max_ir);//Saving Infrared Calibration.. air_speed_bias
  eeprom_write_word((unsigned int*)0x01,air_speed_offset);//Saving Airspeed..

  eeprom_busy_wait();
  eeprom_write_word((unsigned int*)0x0C,vhome_alt);//Saving virtual home altitude

  eeprom_busy_wait();
  eeprom_write_dword((unsigned long*)0x0E,(long)((float)vhome_lat*(float)t7));//Saving virtual home position.

  eeprom_busy_wait();
  eeprom_write_dword((unsigned long*)0x12,(long)((float)vhome_lon*(float)t7));

}

void Get_Home_Position(void)
{    
    eeprom_busy_wait(); 
    home_lat=to_float_6((long)eeprom_read_dword((unsigned long*)0x0E));
    eeprom_busy_wait();
    home_lon=to_float_6((long)eeprom_read_dword((unsigned long*)0x12)); 
    eeprom_busy_wait();
    home_alt=(int)eeprom_read_word((unsigned int*)0x0C);//Restoring launch altitude from eeprom...  
    launch_alt=home_alt; 
}

void print_remzibi_debug(void)
{
  static unsigned long timer2=0;
  
  if(millis()-timer2 > ATTITUDE_RATE_OUTPUT)
  {   
    digitalWrite(13,HIGH);
    Serial.print("$M");
    Serial.print("81"); // Column
    Serial.print("09"); // Row
    Serial.print("A9"); // First Character
    Serial.print("00"); // Last Character
    Serial.print("P:");  
    Serial.print(pitch_set_point);  
    Serial.print(",R:");  
    Serial.print((int)roll_set_point);  
    Serial.print(",T:");
    Serial.print((int)throttle_set_point);
    Serial.print(",S:");
    Serial.print((int)airSpeed());
    Serial.println("        ");

    timer2=millis(); 
    digitalWrite(13,LOW);
  }
}

void print_data_emulator(void)
{
  static unsigned long timer3=0;
  static byte counter;
  
  if(millis()-timer3 > ATTITUDE_RATE_OUTPUT)
  {   
    digitalWrite(13,HIGH);
    Serial.print("!!!");
          Serial.print("LAT:");
      Serial.print((long)((float)lat*(float)t7));
      Serial.print(",LON:");
      Serial.print((long)((float)lon*(float)t7)); //wp_current_lat
      /*
    Serial.print("ASO:");
    Serial.print((int)air_speed_offset);
    Serial.print(",AN3:");
    Serial.print((int)analog3);
    Serial.print(",ASP:");
    Serial.print((int)airSpeed()); 
    */
    Serial.print (",STT:");
    Serial.print((int)Tx_Switch_Status());
    
     Serial.print (",RST:");
     Serial.print((int)wpreset_flag);
     Serial.print (",WLF:");
     Serial.print((int)wplist_flag);
     Serial.print (",CWP:");
     Serial.print((int)current_wp);
     Serial.print (",NWP:");
     Serial.print((int)wp_number);
    
    Serial.print (",WPN:");
    Serial.print((int)last_waypoint);  //This is the target waypoint.
    Serial.print (",DST:");
    Serial.print(wp_distance);
    Serial.print (",RER:");
    Serial.print((int)roll_set_point);
    Serial.print(",THH:");
    Serial.print((int)throttle_set_point);
    Serial.print (",PSET:");
    Serial.print(pitch_set_point);
    Serial.print (",GPS:");
    Serial.print(gpsFix);
    
    Serial.println(",***");
    timer3=millis(); 
    digitalWrite(13,LOW);
  }
}

#endif

#if GPS_PROTOCOL == 0
/****************************************************************
 Parsing stuff for NMEA
 ****************************************************************/
void init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(9600);
  delay(1000);
  Serial.print(LOCOSYS_BAUD_RATE_38400);
  Serial.begin(38400);
  delay(500);
  Serial.print(LOCOSYS_REFRESH_RATE_250);
  delay(500);
  Serial.print(NMEA_OUTPUT_4HZ);
  delay(500);
  Serial.print(SBAS_OFF);
  
  
  /* EM406 example init
  Serial.begin(4800); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(1000);
  Serial.print(SIRF_BAUD_RATE_9600);
 
  Serial.begin(9600);
  delay(1000);
  
  Serial.print(GSV_OFF);
  Serial.print(GSA_OFF);
  
  #if USE_WAAS ==1
  Serial.print(WAAS_ON);
  #else
  Serial.print(WAAS_OFF);
  #endif*/
  
  Wait_GPS_Fix();
}
void fast_init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  //Serial.begin(9600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  Serial.begin(38400);
}

void decode_gps(void)
{
  const char head_rmc[]="GPRMC"; //GPS NMEA header to look for
  const char head_gga[]="GPGGA"; //GPS NMEA header to look for
  
  static unsigned long GPS_timer=0; //used to turn off the LED if no data is received. 
  
  static byte unlock=1; //some kind of event flag
  static byte checksum=0; //the checksum generated
  static byte checksum_received=0; //Checksum received
  static byte counter=0; //general counter

  //Temporary variables for some tasks, specially used in the GPS parsing part (Look at the NMEA_Parser tab)
  unsigned long temp=0;
  unsigned long temp2=0;
  unsigned long temp3=0;


  while(Serial.available() > 0)
  {
    if(unlock==0)
    {
      gps_buffer[0]=Serial.read();//puts a byte in the buffer

      if(gps_buffer[0]=='$')//Verify if is the preamble $
      {
        unlock=1; 
      }
    }
    /*************************************************/
    else
    {
      gps_buffer[counter]=Serial.read();


      if(gps_buffer[counter]==0x0A)//Looks for \F
      {

        unlock=0;


        if (strncmp (gps_buffer,head_rmc,5) == 0)//looking for rmc head....
        {

          /*Generating and parsing received checksum, */
          for(int x=0; x<100; x++)
          {
            if(gps_buffer[x]=='*')
            { 
              checksum_received=strtol(&gps_buffer[x+1],NULL,16);//Parsing received checksum...
              break; 
            }
            else
            {
              checksum^=gps_buffer[x]; //XOR the received data... 
            }
          }

          if(checksum_received==checksum)//Checking checksum
          {
            /* Token will point to the data between comma "'", returns the data in the order received */
            /*THE GPRMC order is: UTC, UTC status ,Lat, N/S indicator, Lon, E/W indicator, speed, course, date, mode, checksum*/
            token = strtok_r(gps_buffer, search, &brkb); //Contains the header GPRMC, not used

            token = strtok_r(NULL, search, &brkb); //UTC Time, not used
            //time=  atol (token);
            token = strtok_r(NULL, search, &brkb); //Valid UTC data? maybe not used... 


            //Longitude in degrees, decimal minutes. (ej. 4750.1234 degrees decimal minutes = 47.835390 decimal degrees)
            //Where 47 are degrees and 50 the minutes and .1234 the decimals of the minutes.
            //To convert to decimal degrees, devide the minutes by 60 (including decimals), 
            //Example: "50.1234/60=.835390", then add the degrees, ex: "47+.835390=47.835390" decimal degrees
            token = strtok_r(NULL, search, &brkb); //Contains Latitude in degrees decimal minutes... 

            //taking only degrees, and minutes without decimals, 
            //strtol stop parsing till reach the decimal point "."  result example 4750, eliminates .1234
            temp=strtol (token,&pEnd,10);

            //takes only the decimals of the minutes
            //result example 1234. 
            temp2=strtol (pEnd+1,NULL,10);

            //joining degrees, minutes, and the decimals of minute, now without the point...
            //Before was 4750.1234, now the result example is 47501234...
            temp3=(temp*10000)+(temp2);


            //modulo to leave only the decimal minutes, eliminating only the degrees.. 
            //Before was 47501234, the result example is 501234.
            temp3=temp3%1000000;


            //Dividing to obtain only the de degrees, before was 4750 
            //The result example is 47 (4750/100=47)
            temp/=100;

            //Joining everything and converting to float variable... 
            //First i convert the decimal minutes to degrees decimals stored in "temp3", example: 501234/600000= .835390
            //Then i add the degrees stored in "temp" and add the result from the first step, example 47+.835390=47.835390 
            //The result is stored in "lat" variable... 
            lat=temp+((float)temp3/600000);


            token = strtok_r(NULL, search, &brkb); //lat, north or south?
            //If the char is equal to S (south), multiply the result by -1.. 
            if(*token=='S'){
              lat=lat*-1;
            }

            //This the same procedure use in lat, but now for Lon....
            token = strtok_r(NULL, search, &brkb);
            temp=strtol (token,&pEnd,10); 
            temp2=strtol (pEnd+1,NULL,10); 
            temp3=(temp*10000)+(temp2);
            temp3=temp3%1000000; 
            temp/=100;
            lon=temp+((float)temp3/600000);

            token = strtok_r(NULL, search, &brkb); //lon, east or west?
            if(*token=='W'){
              lon=lon*-1;
            }

            token = strtok_r(NULL, search, &brkb); //Speed overground?
            ground_speed= atoi(token);

            token = strtok_r(NULL, search, &brkb); //Course?
            ground_course= atoi(token);

            data_update_event|=0x01; //Update the flag to indicate the new data has arrived. 
          }
          checksum=0;
        }//End of the GPRMC parsing

        if (strncmp (gps_buffer,head_gga,5) == 0)//now looking for GPGGA head....
        {
          /*Generating and parsing received checksum, */
          for(int x=0; x<100; x++)
          {
            if(gps_buffer[x]=='*')
            { 
              checksum_received=strtol(&gps_buffer[x+1],NULL,16);//Parsing received checksum...
              break; 
            }
            else
            {
              checksum^=gps_buffer[x]; //XOR the received data... 
            }
          }

          if(checksum_received==checksum)//Checking checksum
          {

            token = strtok_r(gps_buffer, search, &brkb);//GPGGA header, not used anymore
            token = strtok_r(NULL, search, &brkb);//UTC, not used!!
            token = strtok_r(NULL, search, &brkb);//lat, not used!!
            token = strtok_r(NULL, search, &brkb);//north/south, nope...
            token = strtok_r(NULL, search, &brkb);//lon, not used!!
            token = strtok_r(NULL, search, &brkb);//wets/east, nope
            token = strtok_r(NULL, search, &brkb);//Position fix, used!!
            gpsFix =atoi(token); 
            if(gpsFix >=1)
            gpsFix=0;
            else
            gpsFix=1;
            token = strtok_r(NULL, search, &brkb); //sats in use!! Nein...
            token = strtok_r(NULL, search, &brkb);//HDOP, not needed
            token = strtok_r(NULL, search, &brkb);//ALTITUDE, is the only meaning of this string.. in meters of course. 
            alt_MSL=atoi(token);
            if(alt_MSL<0){
              alt_MSL=0;
            }

            if(gpsFix==0x00) digitalWrite(12,HIGH); //Status LED...
            else digitalWrite(12,LOW);

            data_update_event|=0x02; //Update the flag to indicate the new data has arrived.
          }
          checksum=0; //Restarting the checksum
        }

        for(int a=0; a<=counter; a++)//restarting the buffer
        {
          gps_buffer[a]=0;
        } 
        counter=0; //Restarting the counter
        GPS_timer=millis(); //Restarting timer...
      }
      else
      {
        counter++; //Incrementing counter
      }
    }
  }
  
  if(millis() - GPS_timer > 2000){
      digitalWrite(12, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED... 
      gpsFix=1; 
    }  
}
#endif
#if GPS_PROTOCOL == 2
/****************************************************************
 * Here you have all the parsing stuff for uBlox
 ****************************************************************/
//You have to disable all the other string, only leave this ones:

//NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
//NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
/*NAV-STATUS Receiver Navigation Status, PAGE 67 of datasheet
 
 GPSfix Type 
 - 0x00 = no fix
 - 0x01 = dead reckonin
 - 0x02 = 2D-fix
 - 0x03 = 3D-fix
 - 0x04 = GPS + dead re
 - 0x05 = Time only fix
 - 0x06..0xff = reserved*/

//Luckly uBlox has internal EEPROM so all the settings you change will remain forever.  Not like the SIRF modules!

void init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(38400); //Universal Sincronus Asyncronus Receiveing Transmiting 
  Wait_GPS_Fix();
}

void fast_init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(38400); //Universal Sincronus Asyncronus Receiveing Transmiting 
}

/****************************************************************
 * 
 ****************************************************************/
void decode_gps(void)
{
  static unsigned long GPS_timer=0; //used to turn off the LED if no data is received. 
  byte UBX_step=0;
  byte UBX_class=0;
  byte UBX_id=0;
  byte UBX_payload_length_hi=0;
  byte UBX_payload_length_lo=0;
  byte UBX_buffer[40];
  byte UBX_ck_a=0;
  byte UBX_ck_b=0;
  byte j=0;

  if(Serial.available()>50)
  {
    while(Serial.available()>0)//This loop will always run if there is any data available. 
    { 
      switch(UBX_step)//Normally i will start from zero.
      {
      case 0:  
        wait_for_data(1);
        if(Serial.read()==0xB5)UBX_step++; //OH first data packet is correct, so jump to the next step
        break; 
      case 1:  
        wait_for_data(1);
        if(Serial.read()==0x62){
          UBX_step++;
        } //ooh! The second data packet is correct, jump to the step 2
        else {
          UBX_step=0;
        } //Nop, is not correct so restart to step zero and try again.     
        break;
      case 2:
        wait_for_data(4);
        UBX_class=Serial.read(); //Now reading and storing message class (read datasheet if you don't know whats going on) 
        UBX_id=Serial.read(); //Storing ID of the message.
        UBX_payload_length_hi=Serial.read(); //Storing payload high byte
        UBX_payload_length_lo=Serial.read(); //Storing payload low byte, not needed its only for checksum
        checksum(UBX_class); //Generating checksum, read PAGE 63 to know all about UBX checksum
        checksum(UBX_id); //the same//
        checksum(UBX_payload_length_hi); // the same
        checksum(UBX_payload_length_lo); // the same about checksum
        for(int c=0; c<UBX_payload_length_hi; c++) //Saving all the payload in the buffer and continue with the checksum
        {
          wait_for_data(1); //I case there is not data in the buffer i will wait
          UBX_buffer[c]=Serial.read(); //Reading the data from the serial and storing it in the buffer
          checksum(UBX_buffer[c]); //Using the same value i just receive and send it to the checksum "creator"
        }
        wait_for_data(1); //Again waiting for data
        UBX_ck_a=Serial.read(); //Storing the byte received  for checksum a 
        UBX_ck_b=Serial.read(); //Storing the byte received for checksum b 

        if((ck_a=UBX_ck_a)&&(ck_b=UBX_ck_b)) //Verify the received checksum with the generated checksum.. 
        {
          //Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
          //In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
          if(UBX_class==0x01) 
          {            
            switch(UBX_id)//Checking the UBX ID
            {
            case 0x02: //ID NAV-POSLLH 
              //Serial.println((int)UBX_playload_length);
              j=0;
              //iTOW=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); //Storing GPS Millisecond Time of Week
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              iTOW=longUnion.dword;

              //lon=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); //the same with longitude
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              lon=(float)longUnion.dword/(float)10000000;


              //lat=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); //latitude
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              lat=(float)longUnion.dword/(float)10000000;            

              //alt=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++])/10; //Height above Ellipsoid          
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              alt=longUnion.dword;

              //alt_MSL=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++])/10; // Height above the sea level
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              alt_MSL=(float)longUnion.dword/(float)1000;

              //hacc=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++])/10; //Horizontal Accuracy Estimate
              /*
              longUnion.byte[0] = UBX_buffer[j++];
               longUnion.byte[1] = UBX_buffer[j++];
               longUnion.byte[2] = UBX_buffer[j++];
               longUnion.byte[3] = UBX_buffer[j++];
               hacc=(float)longUnion.dword/(float)1000;
               
               //vacc=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++])/10; //Vertical Accuracy Estimate
               longUnion.byte[0] = UBX_buffer[j++];
               longUnion.byte[1] = UBX_buffer[j++];
               longUnion.byte[2] = UBX_buffer[j++];
               longUnion.byte[3] = UBX_buffer[j++];
               vacc=(float)longUnion.dword/(float)1000;*/

              data_update_event|=0x01;
              break;
            case 0x03://ID NAV-STATUS 

              if(UBX_buffer[4] >= 0x03)
              {
                gpsFix=0; //valid position
                digitalWrite(12,HIGH);//Turn LED when gps is fixed. 
              }
              else
              {
                gpsFix=1; //valid position
                digitalWrite(12,LOW);
              }
              break;

            case 0x12:// ID NAV-VELNED 
              j=16;
              //speed_3d=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); //Storing Speed (3-D) 
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              speed_3d=(float)longUnion.dword/(float)100;

              //ground_speed=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); //Ground Speed (2-D) 
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              ground_speed=(float)longUnion.dword/(float)100;

              //heading=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); //Heading 2-D 
              longUnion.byte[0] = UBX_buffer[j++];
              longUnion.byte[1] = UBX_buffer[j++];
              longUnion.byte[2] = UBX_buffer[j++];
              longUnion.byte[3] = UBX_buffer[j++];
              ground_course=(float)longUnion.dword/(float)100000;

              /*
            //sacc=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); //Speed Accuracy Estimate
               longUnion.byte[0] = UBX_buffer[j++];
               longUnion.byte[1] = UBX_buffer[j++];
               longUnion.byte[2] = UBX_buffer[j++];
               longUnion.byte[3] = UBX_buffer[j++];
               sacc=longUnion.dword;
               
               //cacc=join_4_bytes(UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++],UBX_buffer[j++]); // Heading Accuracy Estimate 
               longUnion.byte[0] = UBX_buffer[j++];
               longUnion.byte[1] = UBX_buffer[j++];
               longUnion.byte[2] = UBX_buffer[j++];
               longUnion.byte[3] = UBX_buffer[j++];
               sacc=longUnion.dword;*/

              data_update_event|=0x02; //Update the flag to indicate the new data has arrived.

              break; 
            }
          }   
        }
        //Restarting variables, flushing buffer.. 
        for(int c=0; c<UBX_payload_length_hi; c++)   
        {
          UBX_buffer[c]=0; 
        }
        UBX_step=0;
        ck_a=0;
        ck_b=0;
        GPS_timer=millis(); //Restarting timer...
        break;  
      }
    }
  }
  if(millis() - GPS_timer > 2000){
    digitalWrite(12, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED... 
    gpsFix=1; 
  }  
}

/****************************************************************
 * 
 ****************************************************************/
void checksum(byte data)
{
  ck_a+=data;
  ck_b+=ck_a; 
}
/****************************************************************
 ****************************************************************/
void wait_for_data(byte many)
{
  while(Serial.available()<=many); 
}

#endif




/****************************************************************
 ****************************************************************/
long Get_Relative_Altitude()
{
  long relative=alt_MSL-launch_alt;
  if(relative <= 0)
    relative=0;  
  return relative;
}

/****************************************************************
 * Function that will read and store the current altitude when you switch to autopilot mode.
 ****************************************************************/
unsigned int Hold_Current_Altitude(void)//
{
#if ALT_HOLD_HOME == 1
  return wp_current_alt;
#else
  if(Tx_Switch_Status()==0x00)//Excutes only when we are in manual or in waypoint mode
  {
    hold_Alt=Get_Relative_Altitude();//Updating the current altitude until we switch to RTL
    return hold_Alt;
  }
  else
  {
    if(Tx_Switch_Status()==0x02)//RTL=0x02
    {
      return hold_Alt;
    }
    else
    {
      return wp_current_alt;//WP=0x01
    }
  }
#endif
}


void update_distance(void)
{
  wp_distance = calc_dist(lat, lon, wp_current_lat, wp_current_lon); 
}

/*************************************************************************
 * //Function to calculate the course between two waypoints
 * //I'm using the real formulas--no lookup table fakes!
 *************************************************************************/
int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float calc2;
  float bear_calc;
  float diflon;
  //I've to spplit all the calculation in several steps. If i try it to do it in a single line the arduino will explode.
  flat1=radians(flat1);
  flat2=radians(flat2);

  diflon=radians((flon2)-(flon1));

  calc=sin(diflon)*cos(flat2);
  calc2=cos(flat1)*sin(flat2)-sin(flat1)*cos(flat2)*cos(diflon);

  calc=atan2(calc,calc2);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc;
  }
  return bear_calc;
}
/*************************************************************************
 * //Function to calculate the distance between two waypoints
 * //I'm using  a really good approach
 *************************************************************************/
unsigned int calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters, i love the metric system.. =)
  return dist_calc;
}
 /*****************************************
 * Like Average.. 
 * analog0 = roll infrared.
 * analog1 = pitch infrared. 
 * analog2 = z sensor.
 * analog3 = pressure sensor. 
 *****************************************/
 /* Updated version by Jean-Louis Naudin */
 // 11-08-09 JLN : correction of the air_speed_offset bug which gives -289 (!!!) of airspeed...
 // 11-09-09 JLN : Air_speed_offset bug now corrected, added zasp_req=1 a request flag for Zero Air Speed bias during the catch_analogs Mux
 
 //Activating the ADC interrupts. 
void Analog_Init(void)
{
 ADCSRA|=(1<<ADIE)|(1<<ADEN);
 ADCSRA|= (1<<ADSC);
}
//
int Anal_Read(uint8_t pin)
{
  return analog_buffer[pin];
}
//
void Analog_Reference(uint8_t mode)
{
	analog_reference = mode;
}
//ADC interrupt vector, this piece of
//is executed everytime a convertion is done. 
ISR(ADC_vect)
{
  ADC_flag=0x01;
}
 
/***********************************/ 
void catch_analogs(void)
{ 
  if(ADC_flag==1)
  {
   uint8_t low, high;
   ADC_flag=0; //Restarting flag
   low = ADCL;
   high = ADCH;
   //analog_buffer[MuxSel]=(high << 8) | low;
   switch(MuxSel)
   {
    case 0: analog0=((high << 8)) | low; break; 
    case 1: analog1=((high << 8)) | low; break; 
    case 2: analog2=(((high << 8) | low)*.99)+((float)analog2*.01); sensor_z(); break;
    case 3: analog3=(((high << 8) | low)*.95)+((float)analog3*.05); break;
    case 4: break;
    case 5: analog5=(((high << 8) | low)*.99)+((float)analog5*.01); Batt_Volt=Battery_Voltage(analog5); break; 
   }
   if((zasp_req>=1)&& (MuxSel==3)) zasp_req++; // zero airspeed counter 
   
   if ((zasp_req==5) && (MuxSel==3)) // Zero Airspeed bias after 5 measurements if requested
    { air_speed_offset=(int)analog3;  //air_speed_bias_f;
      zasp_req=0; //zeroe the zaro air speed request flag, air speed bias is now well stored =:-)
    }
   MuxSel++;
   if(MuxSel >=6) MuxSel=0;
   ADMUX = (analog_reference << 6) | (MuxSel & 0x07);
   ADCSRA|= (1<<ADSC);// start the conversion
  //analog3= (float)((float)analog_buffer[3]*.90) + ((float)analog_buffer[3]*.10);
  
  }
}
 int airSpeed(void)
{
 //return constrain(Pressure_MTS((float)(analog3-air_speed_offset))-2,0,500); 
 //COnverting to m/s = sqrt(((((5000mV/1023adc)*(analog3-air_speed_bias))/1 Volt/kP)*2)/1.225 Newtons/m*m)
 return ((int)analog3-air_speed_offset);
}

/*****************************************
 * COnverts the infrared values to degrees, is not very well tuned but it works
 * Is limited to 60 degrees, we dont need more, if the aircraft exceeds 60 degrees it 
 * will try to came back anyway... 
 *****************************************/
 
 int y_axis(void)
 {
   return (int)(((analog0-511)*90)/max_ir);
 }
 
  int x_axis(void)
 {
   return (int)(((analog1-511)*90)/max_ir);
 }
 
int get_roll(void)
{
 
  #if REVERSE_X_SENSOR ==1
  return constrain((x_axis()+y_axis())/2,-60,60); 
  #endif 
  #if REVERSE_X_SENSOR ==0
  return constrain((-x_axis()-y_axis())/2,-60,60);
  #endif  
  //return constrain(y_axis(),-60,60); 
}

int get_pitch(void)
{
  #if REVERSE_X_SENSOR ==1
  return constrain((-x_axis()+y_axis())/2,-60,60);
  #endif
  #if REVERSE_X_SENSOR ==0
  return constrain((+x_axis()-y_axis())/2,-60,60);
  #endif
 //return -constrain(x_axis(),-60,60);   
}

void sensor_z(void)
{
       if(abs(get_roll())<=10)//Checks if the roll is less than 10 degrees to read z sensor
      {
        max_ir=abs(511-analog2);
      } 
}
/**************************************************************
 * Configuring the PWM hadware... If you want to understand this you must read the Data Sheet of atmega168..  
 ***************************************************************/
void Init_servos(void)//This part will configure the PWM to control the servo 100% by hardware, and not waste CPU time.. 
{   
  digitalWrite(10,LOW);//Defining servo output pins
  pinMode(10,OUTPUT);
  digitalWrite(9,LOW);
  pinMode(9,OUTPUT);
  /*Timer 1 settings for fast PWM*/

    //Remember the registers not declared here remains zero by default... 
  TCCR1A =((1<<WGM11)|(1<<COM1B1)|(1<<COM1A1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR1A = 3000; //the period of servo 1, remember 2us resolution, 3000/2 = 1500us the pulse period of the servo...    
  OCR1B = 3000; //the period of servo 2, 3000/2=1500 us, more or less is the central position... 
  ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000, 
  //must be 50hz because is the servo standard (every 20 ms, and 1hz = 1sec) 1000ms/20ms=50hz, elementary school stuff... 

  /*************************************************************/
  /*From here everthing was made to create the throttle servo*/

  TIMSK1 |=(1 << ICIE1); //See page 136, timer 1 interrupt mask

  //Setting up the Timer 2
  TCCR2A = (1<<WGM21); //CTC mode
  TCCR2B =(1<<CS20)|(1<<CS22); //prescaler 128, at 16mhz (128/16)=8, the counter will increment 1 every 8us
  OCR2A = 138; //1500us/8; The top, when the counter reaches the value definied here will execute the interrupt, 187 is the servo centered... 
  //OCR2B = 138;
  //TIMSK2 = (1<<OCIE2A)|(1<<OCIE2B); //interrupt masks for counter A and B
  TIMSK2 = (1<<OCIE2A); //Don't touch!! you have nothing to do here. 
  sei();//Enabling interrupts
}
/*************************************************************************
 * 
 *************************************************************************/

ISR(TIMER1_CAPT_vect)//This is a timer 1 interrupts, executed every 20us 
{
  TCNT2=0; //restarting the counter of timer 2
  PORTB |= 0x01; //Putting the pin high!
  //stabilization_interrupt();
  //PORTB |= 0x09; //Putting the pins high (pint 8 and 11 of arduino)...
}
/*************************************************************************
 * 
 *************************************************************************/
ISR(TIMER2_COMPA_vect ) //Interrupt of timer 2 compare A
{
  PORTB &= 0xFE;//Putting the pin low

  /*
  if((PORTB&0x01)==1)
   PORTB &= 0xFE;
   else
   PORTB |= 0x01;*/
}
/*************************************************************************
 * 
 *************************************************************************/
/*
ISR(TIMER2_COMPB_vect ) //Interrupt of timer 2 compare B, not used now
 {
 //PORTB &= 0xF7;//Putting the pin low
 }*/

/*************************************************************************
 * You must change this if you have another kind of aircraft... 
 *************************************************************************/
void pulse_servos(int roll, int pitch) //Normal mode 
{
  
  #if TEST_SENSORS == 1
  roll= constrain(get_roll(),-45,45);
  pitch = constrain(get_pitch(),-45,45);
  #endif
  
#if MIXING_MODE == 0

  pulse_servo_1(90+(REVERSE_ROLL*roll));
  pulse_servo_2(90+(REVERSE_PITCH*pitch));

#endif
  /*V tail mode*/
#if MIXING_MODE == 1
  pulse_servo_1(90-((REVERSE_PITCH*pitch)+(REVERSE_ROLL*roll)));

  pulse_servo_2(90+((REVERSE_PITCH*pitch)-(REVERSE_ROLL*roll)));

#endif
}
/**************************************************************
 * Function to pulse servo 0
 ***************************************************************/
void pulse_servo_0(long porcent)//Will convert the angle to the equivalent servo position... 
{
  porcent=constrain(porcent,0,100);
  //OCR2A=(((porcent*(long)(throttle_max-throttle_min))/100L)+throttle_min)/8L;
  #if REVERSE_THROTTLE == 1
  porcent=100-porcent;//reversing thrrotle
  #endif 
  OCR2A=(byte)(porcent+137L);
  //Serial.println((porcent+137L));
}

/**************************************************************
 * Function to pulse servo 1
 ***************************************************************/
void pulse_servo_1(long angle)//Will convert the angle to the equivalent servo position... 
{
  angle=constrain(angle,0,180);
  OCR1A=(((angle*(SERVO_AILE_MAX-SERVO_AILE_MIN))/180L)+SERVO_AILE_MIN)*2L;
}

/**************************************************************
 * Function to pulse the servo 2... 
 ***************************************************************/
void pulse_servo_2(long angle)//Will convert the angle to the equivalent servo position... 
{
  angle=constrain(angle,0,180);
  OCR1B=(((angle*(SERVO_ELEV_MAX-SERVO_ELEV_MIN))/180L)+SERVO_ELEV_MIN)*2L; //Scaling
}

/**************************************************************
 * Function to test the servos.. 
 ***************************************************************/
void test_servos(void)
{
  pinMode(4,OUTPUT); //MUX pin 
  digitalWrite(4,HIGH);

  for(int j=0; j<=3; j++)
  {
    switch(j)
    {
    case 0: 
      pulse_servos(-45, 0); 
      break;
    case 1: 
      pulse_servos(45, 0); 
      break;
    case 2: 
      pulse_servos(0, -45); 
      break;
    case 3: 
      pulse_servos(0, 45); 
      break;
    }
    delay(800);
  }
  pulse_servos(0,0);  

  digitalWrite(4,LOW); 
  pinMode(4,INPUT);
}

/**************************************************************
 * Function to read the channels 1 and 2.. 
 ***************************************************************/
int read_Ch1(void)
{
  int temp;
  static int ch;
  temp=pulseIn(2,HIGH, 5000);
  ch= (temp!=0) ? temp : ch; 
  return (REV_FLY_BY_WIRE_CH1*(ch-rx_Ch[0]));
}

int read_Ch2(void)
{
  int temp;
  static int ch;
  temp=pulseIn(3,HIGH, 5000);
  ch= (temp!=0) ? temp : ch; 
  return (REV_FLY_BY_WIRE_CH2*(ch-rx_Ch[1])); //Scary he?
}




/*****************************************
 * Updated version by Jean-Louis Naudin  *
 *****************************************/
// 11-04-09 JLN : Added Rembizi code for GPS Emulator
// 11-09-09 JLN : Air_speed_offset bug now corrected, added zasp_req=1 a request flag for Zero Air Speed bias during the catch_analogs Mux
// 11-11-09 JLN : Added the self-learning mode !!!
// 11-13-09 JLN : Updated the Record_New_Waypoint() function, automatic wplist reset for the first wp
 
void init_ardupilot(void)
{
  pinMode(13,OUTPUT);//LED pin
  
  pinMode(2,INPUT);//Servo input
  pinMode(3,INPUT);//Servo Input 
  pinMode(4,INPUT); //MUX pin
  pinMode(5,INPUT); //Mode pin
  
  digitalWrite(6,HIGH);
  pinMode(6,INPUT); //Remove Before Fly ground
  pinMode(7,OUTPUT); // Mux control pin
  
  #if GPS_PROTOCOL == 3
    digitalWrite(7, LOW);    //Mux switching OFF for GPS Emulator
  #else
      #if SHIELD_VERSION == 0
      digitalWrite(7, LOW); //Mux switching! for Shield v1.0
    #else
      digitalWrite(7, HIGH); //Mux switching! for Shield v2.0
    #endif
  #endif
  
  pinMode(8,OUTPUT);//Servo throtle
  pinMode(11,OUTPUT);//GPS status
  Init_servos(); 
  //Centering Servos
  pulse_servos(0,0); 
  pulse_servo_0(0);
  
  Analog_Reference(0);//Using external analog reference
  Analog_Init();
  
  #if CALIBRATE_SERVOS == 1
  calibrate_servos();
  #endif
  
  rx_Ch[0]=pulseIn(2, HIGH, 20000); //Reading the radio sticks central position...
  rx_Ch[1]=pulseIn(3, HIGH, 20000); //Reading the radio sticks central position...

  zasp_req=1; // Request flag for Zero Air Speed bias
  
  if(digitalRead(6) == LOW) //Verify if the "Remove Before Fly" flag is connected... 
  {
    delay(2000);
    init_gps(); //Initializing GPS
    Wait_GPS_Fix();//Wait GPS fix...
    test_servos(); //testing servos
    Load_Settings();//Loading saved settings
    
    #if FAKE_GPS_LOCK == 0
    while(digitalRead(6) == LOW){ //Loops until we remove the safetly flag
    test_servos(); //Testing servos
    for(int c=0; c<=40; c++)
    {  
    decode_gps();  //Decoding GPS
    catch_analogs(); //Reading Analogs
    }
    }//Loop till we remove the safety flag.. 
    #else
    Serial.println("Warning GPS in fake mode!!!!!!");
    delay(2000);
    #endif
    Save_Launch_Parameters_Flagged();
    current_wp=0; //Restarting waypoint position. 
  }
  else
  { fast_init_gps();
    Load_Settings();
    Restore_Launch_Parameters();
    #if GPS_PROTOCOL == 3   // GPS EMULATOR store the virtual Home position in memory
    Save_Home_Position();    
    Get_Home_Position();
    #endif 

  }
  Serial.println("");
  Serial.println("Ardupilot!!! V2.4.6");
  #if GPS_PROTOCOL == 3 
  Serial.println("WARNING! GPS EMULATOR MODE");
  #endif
  #if PRINT_WAYPOINTS == 1
  print_waypoints();
  #endif
  #if TEST_THROTTLE == 1
  test_throttle();
  #endif
  
  #if PRINT_EEPROM == 1
  Print_EEPROM();
  #endif
}
/*****************************************************************************
 *****************************************************************************/
void Load_Settings(void)
{
  options=(byte)eeprom_read_byte((byte*)0x00);
  air_speed_offset=(int)eeprom_read_word((unsigned int*)0x01); //Restoring airspeed bias
  roll_trim=(byte)eeprom_read_byte((byte*)0x03); //You know like the transmitter trims
  pitch_trim=(byte)eeprom_read_byte((byte*)0x04); 
  max_alt= (int)eeprom_read_word((unsigned int*)0x05);
  max_spd=(int)eeprom_read_word((unsigned int*)0x07);
  wp_number=(byte)eeprom_read_byte((byte*)0x09); //Number of waypoints defined.... Go to to Waypoint tab...
  wp_radius=(byte)eeprom_read_byte((byte*)0x0B); //Radius of the waypoint, normally set as 20 meters.
  //alt_hold=(int)eeprom_read_word((unsigned int*)0x16);  
}

void Save_Launch_Parameters_Flagged(void)
{
  launch_alt=alt_MSL;//Storing launch altitude... 
  //zero_airspeed();
  current_wp=0;

  eeprom_busy_wait(); 
  eeprom_write_byte((byte*)0x0A,current_wp);

  eeprom_busy_wait(); 
  //eeprom_write_word((unsigned int*)0x01,max_ir);//Saving Infrared Calibration.. air_speed_bias
  eeprom_write_word((unsigned int*)0x01,air_speed_offset);//Saving Airspeed..

  eeprom_busy_wait();
  eeprom_write_word((unsigned int*)0x0C,launch_alt);//Saving home altitude

  eeprom_busy_wait();
  eeprom_write_dword((unsigned long*)0x0E,(long)((float)lat*(float)t7));//Saving home position.

  eeprom_busy_wait();
  eeprom_write_dword((unsigned long*)0x12,(long)((float)lon*(float)t7));

}
/****************************************************************
 ****************************************************************/
void Restore_Launch_Parameters(void)
{
  eeprom_busy_wait();
  air_speed_offset=(int)eeprom_read_word((unsigned int*)0x01);//Restoring airspeed bias
  
  #if REMEMBER_LAST_WAYPOINT_MODE == 1
  current_wp=(int)eeprom_read_byte((byte*)0x0A);
  #endif
  
  eeprom_busy_wait();
  launch_alt=(int)eeprom_read_word((unsigned int*)0x0C);//Restoring launch altitude from eeprom...  
}
/****************************************************************
 ****************************************************************/
byte Tx_Switch_Status(void) //Return zero when we are in manual mode, return 2 when autopilot mode 0, return 3 when autopilot mode 1...   
{
  #if RADIO_SWITCH_ACTION == 0
  if(digitalRead(4)==HIGH)  // Not in manual mode
  {
    if(digitalRead(6) == HIGH) // the safetly flag is OFF
    { if(digitalRead(5)==HIGH) // Normal use mode 
      return 0x01; // WP mode 
    else
      return 0x02; // RTL mode
    }
    if(digitalRead(6) == LOW)   // the safetly flag is ON
    { if(digitalRead(5)==HIGH)  
        return 0x03;  // WP reccord mode
    }
  }
  else
    return 0x00; 
  #endif
  #if RADIO_SWITCH_ACTION == 1
  if(digitalRead(4)==HIGH) // Not in manual mode
  {
    if((digitalRead(5)==HIGH)&&(digitalRead(6) == HIGH)) // Normal use mode and the safetly flag OFF
      return 0x02; // RTL mode 
    else
      return 0x01; // wp mode
      
    if(digitalRead(6) == LOW)   // the safetly flag is ON
    { if(digitalRead(5)==HIGH)  // WP learning mode 
        return 0x03;  // WP reccord mode
    }
  }
  else
    return 0x00; 
  #endif
  #if RADIO_SWITCH_ACTION == 2
      return 0x01;  // force wp mode with the Arduino Duemilanove
  #endif
}

/*****************************************************************************
 *****************************************************************************/
float reset(float value)
{
  if(Tx_Switch_Status()==0x00)
    return 0; 
  else
    return value; 
}
/*****************************************************************************
 *****************************************************************************/

void maxis(void)  // Get and store de Max values
{
  if(gpsFix == 0x00) // GPS Locked Status=OK
  {
    if(alt_MSL>max_alt)
    {
      max_alt=alt_MSL;
      eeprom_busy_wait(); 
      eeprom_write_word((unsigned int*)0x05,max_alt);
    }

    if(ground_speed>max_spd)
    {
      max_spd=ground_speed;
      eeprom_busy_wait(); 
      eeprom_write_word((unsigned int*)0x07,max_spd);
    }
  }
}
void print_data(void)
{
  static unsigned long timer1=0;
  static byte counter;
  
  if(millis()-timer1 > ATTITUDE_RATE_OUTPUT)
  {   
     digitalWrite(13,HIGH);
    if(counter >= POSITION_RATE_OUTPUT)//If to reapeat every second.... 
    {
/*      Definitions of the low rate telemetry :
    LAT: Latitude
    LON: Longitude
    SPD: Speed over ground from GPS
    CRT: Climb Rate in M/S
    ALT: Altitude in meters
    ALH: The altitude is trying to hold
    CRS: Course over ground in degrees.
    BER: Bearing is the heading you want to go
    WPN: Waypoint number, where WP0 is home.
    DST: Distance from Waypoint
    BTV: Battery Voltage.
    RSP: Roll setpoint used to debug, (not displayed here).
    
      Definitions of the high rate telemetry :
    ASP: Airspeed, right now is the raw data.
    TTH: Throttle in 100% the autopilot is applying.
    RLL: Roll in degrees + is right - is left
    PCH: Pitch in degrees
    SST: Switch Status, used for debugging, but is disabled in the current version.
*/
      Serial.print("!!!");
      Serial.print("LAT:");
      Serial.print((long)((float)lat*(float)t7));
      Serial.print(",LON:");
      Serial.print((long)((float)lon*(float)t7)); //wp_current_lat
      //Serial.print(",WLA:");
      //Serial.print((long)((float)wp_current_lat*(float)t7));
      //Serial.print(",WLO:");
      //Serial.print((long)((float)wp_current_lon*(float)t7));
      
      Serial.print (",SPD:");
      Serial.print(ground_speed);    
      Serial.print(",CRT:");
      Serial.print(climb_rate);
      Serial.print (",ALT:");
      Serial.print(Get_Relative_Altitude());
      Serial.print (",ALH:");
      Serial.print(Hold_Current_Altitude());
      Serial.print (",MSL:");
      Serial.print(alt_MSL);
      Serial.print (",CRS:");
      Serial.print(ground_course);
      Serial.print (",BER:");
      Serial.print(wp_bearing);
      Serial.print (",WPN:");
      Serial.print((int)last_waypoint);//This the TO waypoint.
      Serial.print (",DST:");
      Serial.print(wp_distance);
      Serial.print (",BTV:");
      Serial.print(Batt_Volt);
      //Serial.print(read_Ch1());
      Serial.print (",RSP:");
      Serial.print(roll_set_point);
 /*
     Serial.print (",GPS:");
     Serial.print(gpsFix);
     Serial.print(read_Ch2());
     Serial.print (",STT:");
     Serial.print((int)Tx_Switch_Status());
     Serial.print (",RST:");
     Serial.print((int)wpreset_flag);
     Serial.print (",WLF:");
     Serial.print((int)wplist_flag);
     Serial.print (",CWP:");
     Serial.print((int)current_wp);
     Serial.print (",NWP:");
     Serial.print((int)wp_number);
 */
      Serial.println(",***");
      counter=0;
      
      //Serial.println(refresh_rate);
      refresh_rate=0;
    }
    else
    {
    counter++;
    
    Serial.print("+++");
/*  Serial.print("ASO:");
    Serial.print((int)air_speed_offset);
    Serial.print(",AN3:");
    Serial.print((int)analog3); */
    Serial.print(",ASP:");
    Serial.print((int)airSpeed());
    Serial.print("THH:");
    Serial.print((int)throttle_set_point);
    Serial.print (",RLL:");
    Serial.print(get_roll());
    //Serial.print(Roll);
    Serial.print (",PCH:");
    Serial.print(get_pitch());
  
    Serial.print (",STT:");
    Serial.print((int)Tx_Switch_Status());

    /*
    Serial.print(",");
    Serial.print ("rER:");
    Serial.print((int)roll_set_point);
    Serial.print (",Mir:");
    Serial.print(max_ir);
    Serial.print(",");
    Serial.print ("CH1:");
    Serial.print(read_Ch1()/16);
    Serial.print(",");
    Serial.print ("CH2:");
    Serial.print(read_Ch2()/16);
    Serial.print (",PSET:");
    Serial.print(pitch_set_point);
    */
    Serial.println(",***");
    }
    timer1=millis(); 
    digitalWrite(13,LOW);
  } 
}

/*****************************************************************************
 *************************** SELF-LEARNING MODE *****************************/

void request_cmd(void)    // Execute the request
{
   if (Tx_Switch_Status()==0x03)
      { 
        if (wplist_flag==0) Record_New_Waypoint(); //If we are not in mode 3 : Create a new waypoint at the current position
      }
  if (Tx_Switch_Status()==0x00) wplist_flag=0;
}


void Record_New_Waypoint(void)   // Create a new waypoint at the current position
{
    unsigned int mem_position; //Memory position on the EEPROM
    
    if (wpreset_flag==0) // Reset the wplist if this is the first recorded wp
      {
        eeprom_busy_wait(); 
        eeprom_write_byte((byte*)0x0A,1); // saving the starting wp
        current_wp=0;
        wp_number=0;
        wpreset_flag=1; // only one zeroe
      }
    
    eeprom_busy_wait(); 
    eeprom_write_byte((byte*)0x0A,1); // saving the starting wp
  
    current_wp++;
        //Storing in eeprom... 
    eeprom_busy_wait(); 
    eeprom_write_byte((byte*)0x09,current_wp); //Save the wp counter...
    
    //Start byte is where the EEPROM location where the waypoints starts, 
    //every waypoint has 10 bytes so i multiply that by the current waypoint + the startbyte = the waypoint EEPROM position.

    eeprom_busy_wait();    
    mem_position = (unsigned int)((unsigned int)Start_Byte+(((unsigned int)current_wp)*10)); 
    
    // saving the current wp position and AGL altitude
    eeprom_busy_wait();
    eeprom_write_dword((unsigned long*)mem_position,(long)((float)lat*(float)t7));
    mem_position+=4;
    eeprom_busy_wait();
    eeprom_write_dword((unsigned long*)mem_position,(long)((float)lon*(float)t7)); 
    mem_position+=4;
    eeprom_busy_wait();
    eeprom_write_word((unsigned int*)mem_position,(int)(alt_MSL-launch_alt));  
    wp_number=current_wp;
    wplist_flag=1;
}

/*****************************************************************************
 * This funtion will switch waypoint's dipending in the radius 
 *****************************************************************************/
// 11-04-09 : JLN added Get_Home_Position(void) for GPS Emulator -> stored into : home_lon, home_lat, home_alt

void Waypoint_Switcher(int distance, byte radius)
{
  static byte locked=0; //This variable allows only one switch... 
  if((distance < radius)&&(locked==0))   
  {
    locked=1; //Locking the switcher until the next waypoints
    current_wp++; //incrementing the waypoint number
    current_wp= (current_wp>wp_number) ? starting_wp : current_wp; //Verefy if the waypoint is more than the waypoints defined, i so restart the counter to zero. 
    //Set_New_Waypoint(current_wp);
    //Storing in eeprom... 
    eeprom_busy_wait(); 
    eeprom_write_byte((byte*)0x0A,current_wp); //Saving currrent waypoints to eempro, right now is not used. 
  }
  else
  {
    if(distance > radius) 
      locked=0; //Reset the lock
  }
}
/*****************************************************************************
 This routine looks more clomplex that it is. Is used to extract the waypoints from the eeprom 
 one by one when needed, so we don't saturated the RAM. 
 *****************************************************************************/
void Set_New_Waypoint(int current_waypoint)
{
    unsigned int mem_position; //Memory position on the EEPROM
  
    if(current_waypoint!=last_waypoint)//I put this to void innecesary access to the eeprom...
    {
      last_waypoint=current_waypoint;
      //Start byte is where the EEPROM location where the waypoints starts, 
      //every waypoint has 10 bytes so i multiply that by the current waypoint + the startbyte = the waypoint EEPROM position.
    mem_position = (unsigned int)((unsigned int)Start_Byte+(((unsigned int)current_waypoint)*10)); 
    eeprom_busy_wait();
    wp_current_lat=to_float_6((long)eeprom_read_dword((unsigned long*)mem_position));
    mem_position+=4;
    eeprom_busy_wait();
    wp_current_lon=to_float_6((long)eeprom_read_dword((unsigned long*)mem_position)); 
    mem_position+=4;
    eeprom_busy_wait();
    wp_current_alt=(int)eeprom_read_word((unsigned int*)mem_position);  
    }
}

/*****************/
/*Debugging Stuff*/
/*****************/

#if PRINT_WAYPOINTS == 1
void print_waypoints(void)
{  
  Serial.print("# of WP: ");
  Serial.println((int)wp_number);
  for(int x=0; x<=(int)wp_number; x++)
  {
    Set_New_Waypoint(x);
    Serial.print(x);
    Serial.print(" Lat: ");
    Serial.print((long)(wp_current_lat*t7));
    Serial.print(" Lon: ");
    Serial.print((long)(wp_current_lon*t7));
    Serial.print(" Alt: ");
    Serial.println(wp_current_alt);
    delay(200);
  }
    #if GPS_PROTOCOL == 3 
    Get_Home_Position();
    Serial.print("Virtual Home");
    Serial.print(" Lat: ");
    Serial.print((long)(home_lat*t7));
    Serial.print(" Lon: ");
    Serial.print((long)(home_lon*t7));
    Serial.print(" Alt: ");
    Serial.println(home_alt);
    #endif
  Set_New_Waypoint(0);
  delay(3000);
}
#endif

#if TEST_THROTTLE == 1
void test_throttle(void)
{
  Serial.println("Ready to test the throttle?");
  delay(1000);
  for(int x=0; x<40; x++)
  {
    pulse_servo_0(x+throttle_dead_zone);
    Serial.print(x);
    delay(100);
  }
  pulse_servo_0(0); 
}
#endif

#if CALIBRATE_SERVOS == 1
void calibrate_servos(void)
{
  while(1)
  {
    Serial.begin(57600);
    pulse_servos(0,0);
    Serial.print(OCR1A/2);
    Serial.print(" ");
    Serial.println(OCR1B/2);
    delay(2000);
    pulse_servos(45,45);
    Serial.print(OCR1A/2);
    Serial.print(" ");
    Serial.println(OCR1B/2);
    delay(2000);
    pulse_servos(-45,-45);
    Serial.print(OCR1A/2);
    Serial.print(" ");
    Serial.println(OCR1B/2);
    delay(2000); 

  }
}
#endif


#if PRINT_EEPROM == 1
void Print_EEPROM(void)
{
  /*
  for(int c=0x00; c<=0x3F; c++)
  {
    Serial.print("0x");
    Serial.print(c,HEX);
    Serial.print("-> ");
    for(int a=0x00; a<=0x0F; a++)
    {
    Serial.print(" 0x");  
    Serial.print(a,HEX);
    Serial.print(":");
    eeprom_busy_wait();
    Serial.print((int)(byte)eeprom_read_byte((byte*)((c*16)+a)),HEX);
    
    }*/
      for(unsigned int c=0x00; c<=0x200; c+=0x02)
  {
    Serial.print(c,HEX);
    Serial.print(":");
    eeprom_busy_wait();
    Serial.print((unsigned int)eeprom_read_word((unsigned int*)c),HEX);
    Serial.println(" ");
  }
  delay(2000);
}
#endif


int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

