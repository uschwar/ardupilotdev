// Released under Creative Commons License 
// Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson (Wired) and Nathan Sindle (SparkFun).
// Version 1.0 for flat board updated by Doug Weibel and Jose Julio

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

#include <avr/eeprom.h>
#include <Wire.h>

//**********************************************************************
//  This section contains USER PARAMETERS !!!
//**********************************************************************

// *** NOTE!   Hardware version - Can be used for v1 (daughterboards) or v2 (flat)
#define BOARD_VERSION 2 // 1 For V1 and 2 for V2

// Ublox gps is recommended!
#define GPS_PROTOCOL 1    // 1 - Ublox,  2 - EM406,  3 - NMEA    We have only tested with Ublox

// Enable Air Start uses Remove Before Fly flag - connection to pin 6 on ArduPilot 
#define ENABLE_AIR_START 1  //  1 if using airstart/groundstart signaling, 0 if not
#define GROUNDSTART_PIN 8    //  Pin number used for ground start signal (recommend 10 on v1 and 8 on v2 hardware)

/*Min Speed Filter for Yaw drift Correction*/
#define SPEEDFILT 2 // >1 use min speed filter for yaw drift cancellation, 0=do not use speed filter

/*For debugging propurses*/
#define PRINT_DEBUG 0   //Will print Debug messages

//OUTPUTMODE=1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 will print accelerometer only data
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_GPS 1     //Will print GPS data

// *** NOTE!   To use AdruIMU with ArduPilot you must select binary output messages
#define PRINT_BINARY 1   //Will print binary message and suppress ASCII messages (above)

// *** NOTE!   Performance reporting is only supported for Ublox.  Set to 0 for others
#define PERFORMANCE_REPORTING 1  //Will include performance reports in the binary output ~ 1/2 min

/* Support for optional magnetometer (1 enabled, 0 dissabled) */
#define USE_MAGNETOMETER 0 // use 1 if you want to make yaw gyro drift corrections using the optional magnetometer                   


//**********************************************************************
//  End of user parameters
//**********************************************************************

#define SOFTWARE_VER "1.6"

// ADC : Voltage reference 3.3v / 10bits(1024 steps) => 3.22mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 3.22mV/ADC step => 330/3.22 = 102.48
// Tested value : 101
#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// LPR530 & LY530 Sensitivity (from datasheet) => 3.33mV/\u00ba/s, 3.22mV/ADC step => 1.03
// Tested values : 0.96,0.96,0.94
#define Gyro_Gain_X 0.92 //X axis Gyro gain
#define Gyro_Gain_Y 0.92 //Y axis Gyro gain
#define Gyro_Gain_Z 0.94 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.015
#define Ki_ROLLPITCH 0.000010
#define Kp_YAW 1.2
//#define Kp_YAW 2.5      //High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005

/*UBLOX Maximum payload length*/
#define UBX_MAXPAYLOAD 56

#define ADC_WARM_CYCLES 75

#define FALSE 0
#define TRUE 1


#include "WProgram.h"
void setup();
void loop();
void startup_ground(void);
void startup_air(void);
void debug_print(char string[]);
void debug_handler(byte message);
void Read_adc_raw(void);
float read_adc(int select);
void Analog_Init(void);
void Analog_Reference(uint8_t mode);
void I2C_Init();
void Compass_Init();
void Read_Compass();
void Compass_Heading();
void Normalize(void);
void Drift_correction(void);
void Accel_adjust(void);
void Matrix_update(void);
void Euler_angles(void);
void init_gps(void);
void decode_gps(void);
void GPS_join_data(void);
void configure_gps(void);
void change_to_sirf_protocol(void);
void init_gps(void);
void decode_gps(void);
void init_gps(void);
void decode_gps(void);
void parse_ubx_gps();
int32_t join_4_bytes(byte Buffer[]);
void checksum(byte ubx_data);
void printdata(void);
void printPerfData(long time);
long convert_to_dec(float x);
float Vector_Dot_Product(float vector1[3],float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
unsigned long DIYmillis();
void DIYdelay(unsigned long ms);
float G_Dt=0.02;    // Integration time (DCM algorithm)

long timeNow=0; // Hold the milliseond value for now
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
boolean groundstartDone = false;    // Used to not repeat ground start

float AN[8]; //array that store the 6 ADC filtered data
float AN_OFFSET[8]; //Array that stores the Offset of the gyros

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

//Magnetometer variables
int magnetom_x;
int magnetom_y;
int magnetom_z;
float MAG_Heading;

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};
float errorCourse=180; 
float COGX=0; //Course overground X axis
float COGY=1; //Course overground Y axis

unsigned int counter=0;
unsigned int cycleCount=0;
byte gyro_sat=0;

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
 
//GPS 

//GPS stuff
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
int gpsFixnew=0; //used to flag when new gps data received - used for binary output message flags
long lat=0; // store the Latitude from the gps to pass to output
long lon=0; // Store the Longitude from the gps to pass to output
long alt_MSL=0; //This is the altitude in millimeters
long iTOW=0; //GPS Millisecond Time of Week
long alt=0;  //Height above Ellipsoid in millimeters
float speed_3d=0; //Speed (3-D)
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=90;//This is the runaway direction of you "plane" in degrees
byte numSV=0; //Number of Sats used. 
float ecefVZ=0; //Vertical Speed in m/s
unsigned long GPS_timer=0;

#if GPS_PROTOCOL == 1
// GPS UBLOX
byte ck_a=0;    // Packet checksum
byte ck_b=0;
byte UBX_step=0;
byte UBX_class=0;
byte UBX_id=0;
byte UBX_payload_length_hi=0;
byte UBX_payload_length_lo=0;
byte UBX_payload_counter=0;
byte UBX_buffer[UBX_MAXPAYLOAD];
byte UBX_ck_a=0;
byte UBX_ck_b=0;
#endif

//ADC variables
volatile uint8_t MuxSel=0;
volatile uint8_t analog_reference = DEFAULT;
volatile uint16_t analog_buffer[8];
volatile uint8_t analog_count[8];


 #if BOARD_VERSION == 1
  uint8_t sensors[6] = {0,2,1,3,5,4};   // Use these two lines for Hardware v1 (w/ daughterboards)
  int SENSOR_SIGN[]= {1,-1,1,-1,1,-1,-1,-1,-1};  //Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
 #endif
 
 #if BOARD_VERSION == 2
  uint8_t sensors[6] = {6,7,3,0,1,2};  // For Hardware v2 flat
  int SENSOR_SIGN[] = {1,-1,-1,1,-1,1,-1,-1,-1};
 #endif
 
 // Performance Monitoring variables
 // Data collected and reported for ~1/2 minute intervals
 #if PERFORMANCE_REPORTING == 1
 int mainLoop_count = 0;              //Main loop cycles since last report
 int G_Dt_max = 0.0;                  //Max main loop cycle time in milliseconds
 byte gyro_sat_count = 0;
 byte adc_constraints = 0;
 byte renorm_sqrt_count = 0;
 byte renorm_blowup_count = 0;
 byte gps_payload_error_count = 0;
 byte gps_checksum_error_count = 0;
 byte gps_pos_fix_count = 0;
 byte gps_nav_fix_count = 0;
 byte gps_messages_sent = 0;
 long perf_mon_timer = 0;
 unsigned int imu_health = 65012;
 #endif

//*****************************************************************************************
void setup()
{ 
  Serial.begin(38400);
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
  pinMode(5,OUTPUT); //Red LED
  pinMode(6,OUTPUT); // Blue LED
  pinMode(7,OUTPUT); // Yellow LED
  pinMode(GROUNDSTART_PIN,INPUT);  // Remove Before Fly flag (pin 6 on ArduPilot)
  digitalWrite(GROUNDSTART_PIN,HIGH);

  
  Analog_Reference(EXTERNAL);//Using external analog reference
  Analog_Init();
  
  debug_print("Welcome...");
  
  #if BOARD_VERSION == 1
  debug_print("You are using Hardware Version 1...");
  #endif 
 
  #if BOARD_VERSION == 2
  debug_print("You are using Hardware Version 2...");
  #endif 
  
  //Serial.print("ArduIMU: v");
  //Serial.println(SOFTWARE_VER);
  debug_handler(0);//Printing version
  
  #if BOARD_VERSION != 1
    #if USE_MAGNETOMETER==1
      debug_handler(3);
      I2C_Init();
      delay(100);
      // Magnetometer initialization
      Compass_Init();
    #endif
  #endif
  

  
  if(ENABLE_AIR_START){
      debug_handler(1);
      //Serial.println("***Air Start");
      startup_air();
  }else{
      //Serial.println("***Ground Start");
      debug_handler(2);
      startup_ground();
  }
 
  
  delay(250);
    
  Read_adc_raw();     // ADC initialization
  timer=DIYmillis();
  delay(20);
}

//***************************************************************************************
void loop() //Main Loop
{
  timeNow = millis();
 
  if((timeNow-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer = timeNow;
#if PERFORMANCE_REPORTING == 1
    mainLoop_count++;
    if (timer-timer_old > G_Dt_max) G_Dt_max = timer-timer_old;
#endif
    G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    if(G_Dt > 1)
      {
        G_Dt = 0;  //keeps dt from blowing up, goes to zero to keep gyros from departing
      }
    
    // *** DCM algorithm
   
    Read_adc_raw();
    
    #if BOARD_VERSION != 1
      #if USE_MAGNETOMETER==1
      if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
        {
        counter=0;
        Read_Compass();    // Read I2C magnetometer
        Compass_Heading(); // Calculate magnetic heading  
        }
      #endif
    #endif
    
    Matrix_update(); 

    Normalize();

    Drift_correction();
   
    Euler_angles();
    
    #if PRINT_BINARY == 1
      printdata(); //Send info via serial
    #endif


    // ***
    
    //Turn on the LED when you saturate any of the gyros.
    if((abs(Gyro_Vector[0])>=ToRad(300))||(abs(Gyro_Vector[1])>=ToRad(300))||(abs(Gyro_Vector[2])>=ToRad(300)))
    {
      gyro_sat=1;
#if PERFORMANCE_REPORTING == 1
      gyro_sat_count++;
#endif
      digitalWrite(5,HIGH);  
    }

    
 cycleCount++;
    if (cycleCount >= 5){ 
        cycleCount = 0;
        // Do these things every 5th time through the main cycle 
        // This section gets called every 1000/(20*5) = 10Hz
        // doing it this way removes the need for another 'millis()' call
      
        decode_gps();
        //Here we will check if we are getting a signal to ground start
        if(digitalRead(GROUNDSTART_PIN) == LOW && groundstartDone == false) startup_ground();
      
      // Display Status on LEDs
      // GYRO Saturation


      if(gyro_sat>=1)
      {
        digitalWrite(5,HIGH); //Turn Red LED when gyro is saturated. 
        if(gyro_sat>=8)  // keep the LED on for 8/10ths of a second


          gyro_sat=0;
        else
          gyro_sat++;
      }
      else
      {
        digitalWrite(5,LOW);
      }
      
      // YAW correction
      if(ground_speed<SPEEDFILT)
      {
        digitalWrite(7,HIGH);    //  Turn on yellow LED if speed too slow and yaw correction supressed
      }
      else
      {
        digitalWrite(7,LOW);
      }
      
      // GPS Fix
      if(gpsFix==0)  // yep its backwards 0 means a good fix in GPS world!
      {
        digitalWrite(6,HIGH);  //Turn Blue LED when gps is fixed. 
      }
      else
      {
        digitalWrite(6,LOW);
      }
      
      // 


      #if !PRINT_BINARY
        printdata(); //Send info via serial
      #endif
    }
     
  
#if PERFORMANCE_REPORTING == 1
    if (timeNow-perf_mon_timer > 30000) 
    {
      printPerfData(timeNow-perf_mon_timer);
      perf_mon_timer=timeNow;
    }
#endif

  }
}

//********************************************************************************
void startup_ground(void)
{
  uint16_t temp=0;
  
  debug_handler(2);
  for(int c=0; c<ADC_WARM_CYCLES; c++)
  { 
    digitalWrite(7,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    delay(50);
    Read_adc_raw();
    digitalWrite(7,HIGH);
    digitalWrite(6,LOW);
    digitalWrite(5,HIGH);
    delay(50);
  }
  digitalWrite(5,LOW);
  digitalWrite(7,LOW);
  
  Read_adc_raw();
  delay(20);
  Read_adc_raw();
  for(int y=0; y<=5; y++)   // Read first initial ADC values for offset.
    AN_OFFSET[y]=AN[y];
  delay(20);
  for(int i=0;i<400;i++)    // We take some readings...
    {
    Read_adc_raw();
    for(int y=0; y<=5; y++)   // Read initial ADC values for offset (averaging).
      AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
    delay(20);
    }
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  for(int y=0; y<=5; y++)
  {
    Serial.println(AN_OFFSET[y]);
    temp = ((AN_OFFSET[y]-200.f)*100.0f);
    eeprom_busy_wait();
    eeprom_write_word((uint16_t *)	(y*2+2), temp);	
  }
  groundstartDone = true;
  Serial.println("***Ground Start complete");
}

//************************************************************************************
void startup_air(void)
{
  uint16_t temp=0;

  for(int y=0; y<=5; y++)
  {
    eeprom_busy_wait();
    temp = eeprom_read_word((uint16_t *)	(y*2+2));
    AN_OFFSET[y] = temp/100.f+200.f;	
    Serial.println(AN_OFFSET[y]);
  }
      Serial.println("***Air Start complete");
}    


void debug_print(char string[])
{
  #if PRINT_DEBUG != 0 
  Serial.print("???");
  Serial.print(string);
  Serial.println("***");
  #endif
}

void debug_handler(byte message)
{
  #if PRINT_DEBUG != 0 
  
  static unsigned long BAD_Checksum=0;
  
   switch(message) 
   {
      case 0:
      Serial.print("???Software Version ");
      Serial.print(SOFTWARE_VER);
      Serial.println("***");
      break;
      
      case 1:
      Serial.println("???Air Start!***");
      break;
      
      case 2:
      Serial.println("???Ground Start!***");
      break;      
      
      case 3:
      Serial.println("???Enabling Magneto...***");
      break;         
     
      case 10:
      BAD_Checksum++;
      Serial.print("???GPS Bad Checksum: "); 
      Serial.print(BAD_Checksum);
      Serial.println("...***");
      break;
      
      default:
      Serial.println("???Invalid debug ID...***");
      break;
   
   }
  #endif
  
}
   
/*
EEPROM memory map

0 0x00		Unused
1 0x01 		..
2 0x02 		AN_OFFSET[0]
3 0x03 		..
4 0x04 		AN_OFFSET[1]
5 0x05 		..
6 0x06 		AN_OFFSET[2]
7 0x07 		..
8 0x08 		AN_OFFSET[3]
9 0x09 		..
10 0x0A		AN_OFFSET[4]
11 0x0B		..
12 0x0C		AN_OFFSET[5]
13 0x0D		..	
14 0x0E		Unused
15 0x0F		..
*/
// We are using an oversampling and averaging method to increase the ADC resolution
// The theorical ADC resolution is now 11.7 bits. Now we store the ADC readings in float format
void Read_adc_raw(void)
{
  int i;
  uint16_t temp1;    
  uint8_t temp2;    
  
  // ADC readings...
  for (i=0;i<6;i++)
    {
      do{
        temp1= analog_buffer[sensors[i]];             // sensors[] maps sensors to correct order 
        temp2= analog_count[sensors[i]];
        } while(temp1 != analog_buffer[sensors[i]]);  // Check if there was an ADC interrupt during readings...
      
      if (temp2>0) AN[i] = float(temp1)/float(temp2);     // Check for divide by zero 
            
    }
  // Initialization for the next readings...
  for (int i=0;i<8;i++){
    do{
      analog_buffer[i]=0;
      analog_count[i]=0;
      } while(analog_buffer[i]!=0); // Check if there was an ADC interrupt during initialization...
  }
}

float read_adc(int select)
{
  float temp;
  if (SENSOR_SIGN[select]<0){
    temp = (AN_OFFSET[select]-AN[select]);
    if (abs(temp)>900) {
#if PRINT_DEBUG != 0
    Serial.print("!!!ADC:1,VAL:");
    Serial.print (temp);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");
#endif
#if PERFORMANCE_REPORTING == 1
    adc_constraints++;   
#endif 
    }
    return constrain(temp,-900,900);             //Throw out nonsensical values
  } else {
    temp = (AN[select]-AN_OFFSET[select]); 
    if (abs(temp)>900) {
#if PRINT_DEBUG != 0
    Serial.print("!!!ADC:2,VAL:");
    Serial.print (temp);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");
#endif    
#if PERFORMANCE_REPORTING == 1
    adc_constraints++; 
#endif
    } 
    return constrain(temp,-900,900);
  }
}

//Activating the ADC interrupts. 
void Analog_Init(void)
{
 ADCSRA|=(1<<ADIE)|(1<<ADEN);
 ADCSRA|= (1<<ADSC);
}

//
void Analog_Reference(uint8_t mode)
{
  analog_reference = mode;
}

//ADC interrupt vector, this piece of code
//is executed everytime a convertion is done. 
ISR(ADC_vect)
{
  volatile uint8_t low, high;
  low = ADCL;
  high = ADCH;

  if(analog_count[MuxSel]<63) {
        analog_buffer[MuxSel] += (high << 8) | low;   // cumulate analog values
        analog_count[MuxSel]++;
  }
  MuxSel++;
  MuxSel &= 0x07;   //if(MuxSel >=8) MuxSel=0;
  ADMUX = (analog_reference << 6) | MuxSel;
  // start the conversion
  ADCSRA|= (1<<ADSC);
}
/* ******************************************************* */
/* I2C HMC5843 magnetometer                                */
/* ******************************************************* */

// Local magnetic declination
// I use this web : http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp
#define MAGNETIC_DECLINATION -6.0    // not used now -> magnetic bearing

int CompassAddress = 0x1E;  //0x3C //0x3D;  //(0x42>>1);

void I2C_Init()
{
  Wire.begin();
}


void Compass_Init()
{
  Wire.beginTransmission(CompassAddress);
  Wire.send(0x02); 
  Wire.send(0x00);   // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission
}

void Read_Compass()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  //Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
    magnetom_x = SENSOR_SIGN[6]*((((int)buff[2]) << 8) | buff[3]);    // X axis (internal sensor y axis)
    magnetom_y = SENSOR_SIGN[7]*((((int)buff[0]) << 8) | buff[1]);    // Y axis (internal sensor x axis)
    magnetom_z = SENSOR_SIGN[8]*((((int)buff[4]) << 8) | buff[5]);    // Z axis
    }
  else
    Serial.println("!ERR: Mag data");
}


void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  // Tilt compensated Magnetic filed X:
  MAG_X = magnetom_x*cos_pitch+magnetom_y*sin_roll*sin_pitch+magnetom_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = magnetom_y*cos_roll-magnetom_z*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
}
/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  boolean problem=FALSE;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= Vector_Dot_Product(&temporary[0][0],&temporary[0][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);
#if PERFORMANCE_REPORTING == 1  
    renorm_sqrt_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???SQT:1,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  } else {
    problem = TRUE;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
	#if PRINT_DEBUG != 0
    Serial.print("???PRB:1,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
      Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[1][0],&temporary[1][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);  
#if PERFORMANCE_REPORTING == 1    
    renorm_sqrt_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???SQT:2,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  } else {
    problem = TRUE;
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???PRB:2,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[2][0],&temporary[2][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);   
#if PERFORMANCE_REPORTING == 1 
    renorm_sqrt_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???SQT:3,RNM:");
    Serial.print (renorm);
    Serial.print (",ERR:");
    Serial.print (error);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  } else {
    problem = TRUE;  
#if PERFORMANCE_REPORTING == 1
    renorm_blowup_count++;
#endif
#if PRINT_DEBUG != 0
    Serial.print("???PRB:3,RNM:");
    Serial.print (renorm);
    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
  
  if (problem) {                // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
      DCM_Matrix[0][0]= 1.0f;
      DCM_Matrix[0][1]= 0.0f;
      DCM_Matrix[0][2]= 0.0f;
      DCM_Matrix[1][0]= 0.0f;
      DCM_Matrix[1][1]= 1.0f;
      DCM_Matrix[1][2]= 0.0f;
      DCM_Matrix[2][0]= 0.0f;
      DCM_Matrix[2][1]= 0.0f;
      DCM_Matrix[2][2]= 1.0f;
      problem = FALSE;  
  }
}

/**************************************************/
void Drift_correction(void)
{
  //Compensation the Roll, Pitch and Yaw drift. 
  float mag_heading_x;
  float mag_heading_y;
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  float Integrator_magnitude;
  float tempfloat;
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
  
  #if PERFORMANCE_REPORTING == 1
    tempfloat = ((Accel_weight - 0.5) * 256.0f);    //amount added was determined to give imu_health a time constant about twice the time constant of the roll/pitch drift correction
    imu_health += tempfloat;
    imu_health = constrain(imu_health,129,65405);
  #endif
  
  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  
  #if USE_MAGNETOMETER==1 
    // We make the gyro YAW drift correction based on compass magnetic heading
    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    
    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
    
    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I   
  #else  // Use GPS Ground course to correct yaw gyro drift
  if(ground_speed>=SPEEDFILT)
  {
    COGX = cos(ToRad(ground_course));
    COGY = sin(ToRad(ground_course));
    errorCourse=(DCM_Matrix[0][0]*COGY) - (DCM_Matrix[1][0]*COGX);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I   
  }
  #endif
  //  Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
  Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I,Omega_I));
  if (Integrator_magnitude > ToRad(300)) {
    Vector_Scale(Omega_I,Omega_I,0.5f*ToRad(300)/Integrator_magnitude);
#if PRINT_DEBUG != 0
    Serial.print("!!!INT:1,MAG:");
    Serial.print (ToDeg(Integrator_magnitude));

    Serial.print (",TOW:");
    Serial.print (iTOW);  
    Serial.println("***");    
#endif
  }
  
  
}
/**************************************************/
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
/**************************************************/

void Matrix_update(void)
{
  Gyro_Vector[0]=Gyro_Scaled_X(read_adc(0)); //gyro x roll
  Gyro_Vector[1]=Gyro_Scaled_Y(read_adc(1)); //gyro y pitch
  Gyro_Vector[2]=Gyro_Scaled_Z(read_adc(2)); //gyro Z yaw
  
  Accel_Vector[0]=read_adc(3); // acc x
  Accel_Vector[1]=read_adc(4); // acc y
  Accel_Vector[2]=read_adc(5); // acc z
  
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  Accel_adjust();    //Remove centrifugal acceleration.
  
 #if OUTPUTMODE==1         
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  #if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
    roll = atan2(Accel_Vector[1],Accel_Vector[2]);    // atan2(acc_y,acc_z)
    pitch = -asin((Accel_Vector[0])/(double)GRAVITY); // asin(acc_x)
    yaw = 0;
  #else
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
  #endif
}

#if GPS_PROTOCOL == 2

#define BUF_LEN 100

// The input buffer
char gps_buffer[BUF_LEN]={
	0x24,0x50,0x53,0x52,0x46,0x31,0x30,0x30,0x2C,0x30,0x2C,0x35,0x37,0x36,0x30,0x30,0x2C,0x38,0x2C,0x31,0x2C,0x30,0x2A,0x33,0x37,0x0D,0x0A};	

// Used to configure Sirf GPS
const byte gps_ender[]={0xB0,0xB3};	

/****************************************************************
 Parsing stuff for SIRF binary protocol. 
 ****************************************************************/
void init_gps(void)
{
	pinMode(2,OUTPUT); //Serial Mux
	digitalWrite(2,HIGH); //Serial Mux
	change_to_sirf_protocol();
	delay(100);//Waits fot the GPS to start_UP
	configure_gps();//Function to configure GPS, to output only the desired msg's
}

void decode_gps(void)
{
	static unsigned long GPS_timer = 0;
	static byte gps_counter = 0; //Another gps counter for the buffer
	static byte GPS_step = 0;
        boolean gps_failure = false;
	static byte gps_ok = 0;//Counter to verify the reciving info
	const byte read_gps_header[]={0xA0,0xA2,0x00,0x5B,0x29};//Used to verify the payload msg header

	if(Serial.available() > 0)//Ok, let me see, the buffer is empty?
	{
		while(Serial.available() < 5){ }
		switch(GPS_step) {
			case 0: //This case will verify the header, to know when the payload will begin 
				while(Serial.available() > 0)	//Loop if data available
				{
					if(Serial.read() == read_gps_header[gps_ok]){ //Checking if the head bytes are equal..
						//if yes increment 1
						gps_ok++; 
					}else{ 
						//Otherwise restart.
						gps_ok = 0; 
					}
					if(gps_ok >= 5) {
						//Ohh 5 bytes are correct, that means jump to the next step, and break the loop
						gps_ok = 0;
						GPS_step++;
						break;
					}
				}
				break; 
			case 1: //Will receive all the payload and join the received bytes... 
				while(Serial.available() < 92){
				}
				gps_counter = 0;
				memset(gps_buffer,0,sizeof(gps_buffer));
				
				while(Serial.available() > 0){
					//Read data and store it in the temp buffer
					byte b1 = Serial.read();
					byte b2 = gps_buffer[gps_counter-1];
					// gps_ender[]={0xB0,0xB3};	

					if((b1 == gps_ender[1]) && (b2 == gps_ender[0])){
						GPS_step = 0;
						gps_counter = 0;
						gpsFix = gps_buffer[1];
						
						if(gpsFix == 0x00){
							// GPS signal is error free
							// ------------------------
							digitalWrite(6,HIGH);
							GPS_timer = millis();
                                                        gpsFixnew=1;  //new information available flag for binary message
							
							//Parse the data
							GPS_join_data(); 

						} else {
							// GPS has returned an error code
							// ------------------------------
							gpsFix = 0x01;          // In GPS language a good fix = 0,  bad = 1
							digitalWrite(6,LOW);
						}
						
						break;
					}else{
						gps_buffer[gps_counter] = b1;
						gps_counter++;
						
						if (gps_counter >= BUF_LEN){
							Serial.flush();
							break;
						}
					}
				}
				break;
		}
	}
	
	if(millis() - GPS_timer > 2000){
		digitalWrite(6, LOW);	//If we don't receive any byte in two seconds turn off gps fix LED... 
		gpsFix = 0x01;
	}
}

void GPS_join_data(void)
{
	// Read bytes and combine them with Unions
	// ---------------------------------------
	byte j = 22;
	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];
	lat					= longUnion.dword;		// latitude * 10,000,000


	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];
	lon					= longUnion.dword;		// longitude * 10,000,000

	j = 34;
	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];	
	alt_MSL				= longUnion.dword * 10;		// alt in meters * 1000

	j = 39;
	intUnion.byte[1] 	= gps_buffer[j++];
	intUnion.byte[0] 	= gps_buffer[j++];
	speed_3d			= (float)intUnion.word / 100.0;		// meters/second    We only get ground speed but store it as speed_3d for use in DCM
	
	//iTOW
	
		intUnion.byte[1] 	= gps_buffer[j++];
		intUnion.byte[0] 	= gps_buffer[j++];
		ground_course 		= (float)intUnion.word / 100.0;		// degrees
		ground_course 		= abs(ground_course);	//The GPS has a BUG sometimes give you the correct value but negative, weird!! 	
	
	// clear buffer
	// -------------
	memset(gps_buffer,0,sizeof(gps_buffer));
}


void configure_gps(void)
{
	const byte gps_header[]={
		0xA0,0xA2,0x00,0x08,0xA6,0x00			};//Used to configure Sirf GPS
	const byte gps_payload[]={
		0x02,0x04,0x07,0x09,0x1B			};//Used to configure Sirf GPS
	const byte gps_checksum[]={
		0xA8,0xAA,0xAD,0xAF,0xC1			};//Used to configure Sirf GPS
	const byte cero = 0x00;//Used to configure Sirf GPS

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
				Serial.print(byte(cero)); //Prints 6 zeros
			} 
			Serial.print(byte(gps_checksum[x])); //Print the Checksum
			Serial.print(byte(gps_ender[0]));	//Print the Ender of the string, is same on all msg's. 
			Serial.print(byte(gps_ender[1]));	//ender	
		}
	}	
}

void change_to_sirf_protocol(void)
{
	Serial.begin(4800); //First try in 4800
	delay(300);
	for (byte x=0; x<=28; x++)
	{
		Serial.print(byte(gps_buffer[x]));//Sending special bytes declared at the beginning 
	}	
	delay(300);
	Serial.begin(9600); //Then try in 9600 
	delay(300);
	for (byte x=0; x<=28; x++)
	{
		Serial.print(byte(gps_buffer[x]));
	}
	Serial.begin(57600); //Universal Syncronus Asyncronus Receiveing Transmiting
}

#endif

#if GPS_PROTOCOL == 3

//*********************************************************************************************
//  You may need to insert parameters appropriate for your gps from this list into init_gps()
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
//**************************************************************************************************************


/****************************************************************
 Parsing stuff for NMEA
 ****************************************************************/
#define BUF_LEN 200

// GPS Pointers
char *token;
char *search = ",";
char *brkb, *pEnd;
char gps_buffer[BUF_LEN]; //The traditional buffer.

void init_gps(void)
{
	pinMode(2,OUTPUT); //Serial Mux
	digitalWrite(2,HIGH); //Serial Mux
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
	
	#if USE_WAAS == 1
	Serial.print(WAAS_ON);
	#else
	Serial.print(WAAS_OFF);
	#endif
*/
	
}

void decode_gps(void)
{
	const char head_rmc[]="GPRMC"; //GPS NMEA header to look for
	const char head_gga[]="GPGGA"; //GPS NMEA header to look for
	
	static unsigned long GPS_timer = 0; //used to turn off the LED if no data is received. 
	
	static byte unlock = 1; //some kind of event flag
	static byte checksum = 0; //the checksum generated
	static byte checksum_received = 0; //Checksum received
	static byte counter = 0; //general counter

	//Temporary variables for some tasks, specially used in the GPS parsing part (Look at the NMEA_Parser tab)
	unsigned long temp = 0;
	unsigned long temp2 = 0;
	unsigned long temp3 = 0;


	while(Serial.available() > 0)
	{
		if(unlock == 0)
		{
			gps_buffer[0] = Serial.read();//puts a byte in the buffer

			if(gps_buffer[0]=='$')//Verify if is the preamble $
			{
				counter 	= 0;
				checksum 	= 0;
				unlock		= 1;
			}
		} else {
			gps_buffer[counter] = Serial.read();

			if(gps_buffer[counter] == 0x0A)//Looks for \F
			{
				unlock = 0;

				if (strncmp (gps_buffer, head_rmc, 5) == 0)//looking for rmc head....
				{

					/*Generating and parsing received checksum, */
					for(int x=0; x<100; x++)
					{
						if(gps_buffer[x]=='*')
						{ 
							checksum_received = strtol(&gps_buffer[x + 1], NULL, 16);//Parsing received checksum...
							break; 
						}
						else
						{
							checksum ^= gps_buffer[x]; //XOR the received data... 
						}
					}

					if(checksum_received == checksum)//Checking checksum
					{
						/* Token will point to the data between comma "'", returns the data in the order received */
						/*THE GPRMC order is: UTC, UTC status , Lat, N/S indicator, Lon, E/W indicator, speed, course, date, mode, checksum*/
						token = strtok_r(gps_buffer, search, &brkb); //Contains the header GPRMC, not used

						token = strtok_r(NULL, search, &brkb); //UTC Time, not used
						//time=	atol (token);
						token = strtok_r(NULL, search, &brkb); //Valid UTC data? maybe not used... 


						//Longitude in degrees, decimal minutes. (ej. 4750.1234 degrees decimal minutes = 47.835390 decimal degrees)
						//Where 47 are degrees and 50 the minutes and .1234 the decimals of the minutes.
						//To convert to decimal degrees, devide the minutes by 60 (including decimals), 
						//Example: "50.1234/60=.835390", then add the degrees, ex: "47+.835390 = 47.835390" decimal degrees
						token = strtok_r(NULL, search, &brkb); //Contains Latitude in degrees decimal minutes... 

						//taking only degrees, and minutes without decimals, 
						//strtol stop parsing till reach the decimal point "."	result example 4750, eliminates .1234
						temp = strtol (token, &pEnd, 10);

						//takes only the decimals of the minutes
						//result example 1234. 
						temp2 = strtol (pEnd + 1, NULL, 10);

						//joining degrees, minutes, and the decimals of minute, now without the point...
						//Before was 4750.1234, now the result example is 47501234...
						temp3 = (temp * 10000) + (temp2);


						//modulo to leave only the decimal minutes, eliminating only the degrees.. 
						//Before was 47501234, the result example is 501234.
						temp3 = temp3 % 1000000;


						//Dividing to obtain only the de degrees, before was 4750 
						//The result example is 47 (4750/100 = 47)
						temp /= 100;

						//Joining everything and converting to * 10,000,000 ... 
						//First i convert the decimal minutes to degrees decimals stored in "temp3", example: 501234/600000 =.835390
						//Then i add the degrees stored in "temp" and add the result from the first step, example 47+.835390 = 47.835390 
						//The result is stored in "lat" variable... 
						//**This is all changed in this case to be a long integer which is decimal degrees * 10**7

						lat	= (temp * 10000000) + ((temp3 *100) / 6);

						token = strtok_r(NULL, search, &brkb); //lat, north or south?
						//If the char is equal to S (south), multiply the result by -1.. 
						if(*token == 'S'){
							lat *= -1;
						}

						//This the same procedure use in lat, but now for Lon....
						token = strtok_r(NULL, search, &brkb);
						temp = strtol (token,&pEnd, 10); 
						temp2 = strtol (pEnd + 1, NULL, 10); 
						temp3 = (temp * 10000) + (temp2);
						temp3 = temp3%1000000; 
						temp/= 100;
						lon	= (temp * 10000000) + ((temp3 * 100) / 6);

						token = strtok_r(NULL, search, &brkb); //lon, east or west?
						if(*token == 'W'){
							lon *= -1;
						}

						token = strtok_r(NULL, search, &brkb); 	//Speed over ground
						speed_3d = (float)atoi(token);					// We only get ground speed but store it as speed_3d for use in DCM

						token = strtok_r(NULL, search, &brkb); //Course
						ground_course = (float)atoi(token);
						
						gpsFixnew=1;  			//new information available flag for binary message

					}
					checksum = 0;
				}//End of the GPRMC parsing

				if (strncmp (gps_buffer, head_gga, 5) == 0)//now looking for GPGGA head....
				{
					/*Generating and parsing received checksum, */
					for(int x = 0; x<100; x++)
					{
						if(gps_buffer[x]=='*')
						{ 
							checksum_received = strtol(&gps_buffer[x + 1], NULL, 16);//Parsing received checksum...
							break; 
						}
						else
						{
							checksum^= gps_buffer[x]; //XOR the received data... 
						}
					}

					if(checksum_received== checksum)//Checking checksum
					{
						//strcpy(gps_GGA,gps_buffer);

						token = strtok_r(gps_buffer, search, &brkb);//GPGGA header, not used anymore
						token = strtok_r(NULL, search, &brkb);//UTC, not used!!
						token = strtok_r(NULL, search, &brkb);//lat, not used!!
						token = strtok_r(NULL, search, &brkb);//north/south, nope...
						token = strtok_r(NULL, search, &brkb);//lon, not used!!
						token = strtok_r(NULL, search, &brkb);//wets/east, nope
						token = strtok_r(NULL, search, &brkb);//Position fix, used!!
						
						if(atoi(token) >= 1){
							gpsFix = 0x00;			// gpsFix = 0 means valid fix
						}else{
							gpsFix = 0x01;
						}
						token = strtok_r(NULL, search, &brkb); //sats in use!! Nein...
						token = strtok_r(NULL, search, &brkb);//HDOP, not needed
						token = strtok_r(NULL, search, &brkb);//ALTITUDE, is the only meaning of this string.. in meters of course. 
						alt_MSL = abs(atoi(token)) * 1000;
						
						if(gpsFix == 0x00) digitalWrite(6, HIGH); //Status LED...
						else digitalWrite(6, LOW);
					}
					checksum = 0; //Restarting the checksum
				}

				for(int a = 0; a<= counter; a++)//restarting the buffer
				{
					gps_buffer[a]= 0;
				} 
				counter = 0; //Restarting the counter
				GPS_timer = millis(); //Restarting timer...
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
		digitalWrite(6, LOW);	//If we don't receive any byte in two seconds turn off gps fix LED... 
		gpsFix = 0x01;
		gpsFixnew = 0;
	}
}
#endif

#if GPS_PROTOCOL == 1

/****************************************************************
 * Here you have all the parsing stuff for uBlox
 ****************************************************************/
 // Code from Jordi, modified by Jose
 
 //You have to disable all the other string, only leave this ones:
 
 //NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
 //NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
 //NAV-STATUS Receiver Navigation Status, PAGE 67 of datasheet
 //NAV-SOL Navigation Solution Information, PAGE 72 of datasheet
 
 
 // Baud Rate:38400

/* 
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
  Serial.begin(38400); 
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
}  
/****************************************************************
 * 
 ****************************************************************/
// optimization : This code doesn`t wait for data, only proccess the data available in the Serial buffer
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_ubx_gps() to parse and update the GPS info.
void decode_gps(void)
{ 
  byte data;
  int numc;
  
  numc = Serial.available();
  if (numc > 0) {
    for (int i=0;i<numc;i++)  // Process bytes received
    {
      data = Serial.read();
      switch(UBX_step)     //Normally we start from zero. This is a state machine
      {
      case 0:  
        if(data==0xB5)  // UBX sync char 1
          UBX_step++;   //first data packet is correct, so jump to the next step
        break; 
      case 1:  
        if(data==0x62)  // UBX sync char 2
          UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
        else 
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
        break;
      case 2:
        UBX_class=data;
        checksum(UBX_class);
        UBX_step++;
        break;
      case 3:
        UBX_id=data;
        checksum(UBX_id);
        UBX_step++;
        break;
      case 4:
        UBX_payload_length_hi=data;
        checksum(UBX_payload_length_hi);
        UBX_step++;
        if (UBX_payload_length_hi>UBX_MAXPAYLOAD)
        {
#if PRINT_DEBUG != 0
          Serial.print("???GPS:1,BAD Payload length:");   
          Serial.print(UBX_payload_length_hi,DEC);
          Serial.println("***");    
#endif
          UBX_step=0;   //Bad data, so restart to step zero and try again.     
          UBX_payload_counter=0;
          ck_a=0;
          ck_b=0;
#if PERFORMANCE_REPORTING == 1
          gps_payload_error_count++;
#endif
        }
        break;
      case 5:
        UBX_payload_length_lo=data;
        checksum(UBX_payload_length_lo);
        UBX_step++;
        break;
      case 6:         // Payload data read...
					// We need to process the data byte before we check the number of bytes so far
					/*UBX_buffer[UBX_payload_counter] = data;
					checksum(data);
					UBX_payload_counter++;
                                         
					if (UBX_payload_counter < UBX_payload_length_hi) {
						// We stay in this state until we reach the payload_length
					} else {
						UBX_step++; // payload_length reached - next byte is checksum
					}*/
                                        
                                        if (UBX_payload_counter < UBX_payload_length_hi) 
                                        {
                                          UBX_buffer[UBX_payload_counter] = data;
                                          checksum(data);
                                          UBX_payload_counter++;
                                          
                                        if (UBX_payload_counter==UBX_payload_length_hi)
                                        UBX_step++;
                                        
                                        }
                                        
                                        
					break;
      case 7:
        UBX_ck_a=data;   // First checksum byte
        UBX_step++;
        break;
      case 8:
        UBX_ck_b=data;   // Second checksum byte
       
	  // We end the GPS read...
        if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))   // Verify the received checksum with the generated checksum.. 
	  	parse_ubx_gps();               // Parse new GPS packet...

        else
        {
#if PERFORMANCE_REPORTING == 1          
            gps_checksum_error_count++;
#endif
            debug_handler(10);    
            Serial.print("???");
            Serial.print((int)UBX_ck_a);
            Serial.print("-");
            Serial.print((int)ck_a);
            Serial.println("***");
            
            Serial.print("???");
            Serial.print((int)UBX_ck_b);
            Serial.print("-");
            Serial.print((int)ck_b);
            Serial.println("***");
            
        }

        // Variable initialization
        UBX_step=0;
        UBX_payload_counter=0;
        ck_a=0;
        ck_b=0;
        GPS_timer=DIYmillis(); //Restarting timer...
        break;
	}
    }    // End for...
  }  
  if(DIYmillis() - GPS_timer > 2000){
      digitalWrite(6, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED...
      debug_print("Yeah, your GPS is disconnected"); 
      gpsFix=1; 
  }
  
}

/****************************************************************
 * 
 ****************************************************************/
void parse_ubx_gps()
{
  int j;
//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
  if(UBX_class==0x01) 
  {
    switch(UBX_id)//Checking the UBX ID
    {
      case 0x02: //ID NAV-POSLLH 
#if PERFORMANCE_REPORTING == 1
         gps_pos_fix_count++;
#endif
        j=0;
        iTOW = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        lon = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        lat = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        alt = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        alt_MSL = join_4_bytes(&UBX_buffer[j]);
        j+=4;
        /*
        hacc = (float)join_4_bytes(&UBX_buffer[j])/1000.0;
        j+=4;
        vacc = (float)join_4_bytes(&UBX_buffer[j])/1000.0;
        j+=4;
        */
      break;
    case 0x03://ID NAV-STATUS 
       if((UBX_buffer[4] >= 0x03)&&(UBX_buffer[5]&0x01))
        {
          gpsFix=0; //valid position
          gpsFixnew=1;  //new information available flag for binary message
          digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
          GPS_timer=DIYmillis(); //Restarting timer...
        }
        else
        {
          gpsFix=1; //invalid position
          digitalWrite(6,LOW);
        }
      break;
      
    case 0x06://ID NAV-SOL
       if((UBX_buffer[10] >= 0x03)&&(UBX_buffer[11]&0x01))
        {
          gpsFix=0; //valid position
          gpsFixnew=1;  //new information available flag for binary message
          digitalWrite(6,HIGH);  //Turn LED when gps is fixed. 
          GPS_timer=DIYmillis(); //Restarting timer...
        }
        else
        {
          gpsFix=1; //invalid position
          digitalWrite(6,LOW);
        }
        
        ecefVZ=(float)join_4_bytes(&UBX_buffer[36])/100; //Vertical Speed
        
        numSV=UBX_buffer[47]; //Number of sats...     
    break;

    case 0x12:// ID NAV-VELNED  
#if PERFORMANCE_REPORTING == 1
         gps_nav_fix_count++;
#endif
      j=16;
      speed_3d = (float)join_4_bytes(&UBX_buffer[j])/100.0; // m/s
      j+=4;
      ground_speed = (float)join_4_bytes(&UBX_buffer[j])/100.0; // Ground speed 2D
      j+=4;
      ground_course = (float)join_4_bytes(&UBX_buffer[j])/100000.0; // Heading 2D
      j+=4;
      /*
      sacc = join_4_bytes(&UBX_buffer[j]) // Speed accuracy
      j+=4;
      headacc = join_4_bytes(&UBX_buffer[j]) // Heading accuracy
      j+=4;
      */
      break; 
      }
    }   
}


/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
int32_t join_4_bytes(byte Buffer[])
{
  longUnion.byte[0] = *Buffer;
  longUnion.byte[1] = *(Buffer+1);
  longUnion.byte[2] = *(Buffer+2);
  longUnion.byte[3] = *(Buffer+3);
  return(longUnion.dword);
}

/****************************************************************
 * 
 ****************************************************************/
void checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}


#endif

void printdata(void)
{    

#if PRINT_BINARY != 1  //Print either Ascii or binary messages

     Serial.print("!!!VER:");
     Serial.print(SOFTWARE_VER);  //output the software version
     Serial.print(",");
	#if PRINT_ANALOGS==1
      Serial.print("AN0:");
      Serial.print(read_adc(0)); //Reversing the sign. 
      Serial.print(",AN1:");
      Serial.print(read_adc(1));
      Serial.print(",AN2:");
      Serial.print(read_adc(2));  
      Serial.print(",AN3:");
      Serial.print(read_adc(3));
      Serial.print (",AN4:");
      Serial.print(read_adc(4));
      Serial.print (",AN5:");
      Serial.print(read_adc(5));
      Serial.print (",");
	#endif
      
      #if PRINT_DCM == 1
      Serial.print ("EX0:");
      Serial.print(convert_to_dec(DCM_Matrix[0][0]));
      Serial.print (",EX1:");
      Serial.print(convert_to_dec(DCM_Matrix[0][1]));
      Serial.print (",EX2:");
      Serial.print(convert_to_dec(DCM_Matrix[0][2]));
      Serial.print (",EX3:");
      Serial.print(convert_to_dec(DCM_Matrix[1][0]));
      Serial.print (",EX4:");
      Serial.print(convert_to_dec(DCM_Matrix[1][1]));
      Serial.print (",EX5:");
      Serial.print(convert_to_dec(DCM_Matrix[1][2]));
      Serial.print (",EX6:");
      Serial.print(convert_to_dec(DCM_Matrix[2][0]));
      Serial.print (",EX7:");
      Serial.print(convert_to_dec(DCM_Matrix[2][1]));
      Serial.print (",EX8:");
      Serial.print(convert_to_dec(DCM_Matrix[2][2]));
      Serial.print (",");
      #endif
      #if PRINT_EULER == 1
      Serial.print("RLL:");
      Serial.print(ToDeg(roll));
      Serial.print(",PCH:");
      Serial.print(ToDeg(pitch));
      Serial.print(",YAW:");
      Serial.print(ToDeg(yaw));
      Serial.print(",IMUH:");
      Serial.print((imu_health>>8)&0xff);
      Serial.print (",");
      #endif
      
      #if USE_MAGNETOMETER == 1
      Serial.print("MGX:");
      Serial.print(magnetom_x);
      Serial.print (",MGY:");
      Serial.print(magnetom_y);
      Serial.print (",MGZ:");
      Serial.print(magnetom_z);
      Serial.print (",MGH:");
      Serial.print(MAG_Heading);
      Serial.print (",");
      #endif
      
      
      #if PRINT_GPS == 1
      if(gpsFixnew==1)
      {
      gpsFixnew=0;
      Serial.print("LAT:");
      Serial.print(lat);
      Serial.print(",LON:");
      Serial.print(lon);
      Serial.print(",ALT:");
      Serial.print(alt_MSL/1000);
      Serial.print(",COG:");
      Serial.print((ground_course));
      Serial.print(",SOG:");
      Serial.print(ground_speed);
      Serial.print(",FIX:");
      Serial.print((int)gpsFix);
      Serial.print(",EVZ:"); //Vertical Speed
      Serial.print(ecefVZ);
      Serial.print(",SAT:"); 
      Serial.print((int)numSV);
      
      Serial.print (",");
      #if PERFORMANCE_REPORTING == 1
        gps_messages_sent++;
      #endif
      }
      #endif
      
     Serial.print("TOW:");
     Serial.print(iTOW);
      
     Serial.println("***");    

#else
      //  This section outputs binary data messages
      //  Conforms to new binary message standard (12/31/09)
      byte IMU_buffer[20];
      int tempint;
      int ck;
      long templong;
      byte IMU_ck_a=0;
      byte IMU_ck_b=0;
      
	//  This section outputs the gps binary message when new gps data is available
	if(gpsFixnew==1)
	{
	 	#if PERFORMANCE_REPORTING == 1
			gps_messages_sent++;
		#endif
		gpsFixnew=0;
		Serial.print("DIYd");  // This is the message preamble
                IMU_buffer[0]=0x13;
                ck=19;
		IMU_buffer[1] = 0x03;      

		templong = lon; //Longitude *10**7 in 4 bytes
		IMU_buffer[2]=templong&0xff;
		IMU_buffer[3]=(templong>>8)&0xff;
		IMU_buffer[4]=(templong>>16)&0xff;
		IMU_buffer[5]=(templong>>24)&0xff;
      
		templong = lat; //Latitude *10**7 in 4 bytes
		IMU_buffer[6]=templong&0xff;
		IMU_buffer[7]=(templong>>8)&0xff;
		IMU_buffer[8]=(templong>>16)&0xff;
		IMU_buffer[9]=(templong>>24)&0xff;
      
		tempint=alt_MSL / 100;   // Altitude MSL in meters * 10 in 2 bytes
		IMU_buffer[10]=tempint&0xff;
		IMU_buffer[11]=(tempint>>8)&0xff;
      
		tempint=speed_3d*100;   // Speed in M/S * 100 in 2 bytes
		IMU_buffer[12]=tempint&0xff;
		IMU_buffer[13]=(tempint>>8)&0xff;
        
		tempint=ground_course*100;   // course in degreees * 100 in 2 bytes
		IMU_buffer[14]=tempint&0xff;
		IMU_buffer[15]=(tempint>>8)&0xff;
        
		IMU_buffer[16]=iTOW&0xff;
		IMU_buffer[17]=(iTOW>>8)&0xff;
		IMU_buffer[18]=(iTOW>>16)&0xff;
		IMU_buffer[19]=(iTOW>>24)&0xff;

        	IMU_buffer[20]=(imu_health>>8)&0xff;

		for (int i=0;i<ck+2;i++) Serial.print (IMU_buffer[i]);
		
		for (int i=0;i<ck+2;i++) {
			IMU_ck_a+=IMU_buffer[i];  //Calculates checksums
			IMU_ck_b+=IMU_ck_a;       
		}
      	Serial.print(IMU_ck_a);
      	Serial.print(IMU_ck_b);
  
	} else {
      
	  // This section outputs the IMU orientatiom message
          Serial.print("DIYd");  // This is the message preamble
          IMU_buffer[0]=0x06;
          ck=6;
          IMU_buffer[1] = 0x02;      

          tempint=ToDeg(roll)*100;  //Roll (degrees) * 100 in 2 bytes
          IMU_buffer[2]=tempint&0xff;
          IMU_buffer[3]=(tempint>>8)&0xff;
      
          tempint=ToDeg(pitch)*100;   //Pitch (degrees) * 100 in 2 bytes
          IMU_buffer[4]=tempint&0xff;
          IMU_buffer[5]=(tempint>>8)&0xff;
      
          tempint=ToDeg(yaw)*100;  //Yaw (degrees) * 100 in 2 bytes
          IMU_buffer[6]=tempint&0xff;
          IMU_buffer[7]=(tempint>>8)&0xff;
      
          for (int i=0;i<ck+2;i++) Serial.print (IMU_buffer[i]);
	  
          for (int i=0;i<ck+2;i++) {
              IMU_ck_a+=IMU_buffer[i];  //Calculates checksums
              IMU_ck_b+=IMU_ck_a;       
          }
          Serial.print(IMU_ck_a);
          Serial.print(IMU_ck_b);
          
    }

         
#endif  
}

void printPerfData(long time)
{

// This function outputs a performance monitoring message (used every 30 seconds) 
//  Can be either binary or human readable
#if PRINT_BINARY == 1
      byte IMU_buffer[30];
      int ck;
      byte IMU_ck_a=0;
      byte IMU_ck_b=0;
      
      	Serial.print("DIYd");  // This is the message preamble
		IMU_buffer[0]=0x11;
		ck=17;
      	IMU_buffer[1] = 0x0a;      

        	//Time for this reporting interval in millisecons
        	IMU_buffer[2]=time&0xff;
        	IMU_buffer[3]=(time>>8)&0xff;
        	IMU_buffer[4]=(time>>16)&0xff;
        	IMU_buffer[5]=(time>>24)&0xff;
      
        	IMU_buffer[6]=mainLoop_count&0xff;
        	IMU_buffer[7]=(mainLoop_count>>8)&0xff;
        
        	IMU_buffer[8]=G_Dt_max&0xff;
        	IMU_buffer[9]=(G_Dt_max>>8)&0xff;
      
        	IMU_buffer[10]=gyro_sat_count;
        	IMU_buffer[11]=adc_constraints;
        	IMU_buffer[12]=renorm_sqrt_count;
        	IMU_buffer[13]=renorm_blowup_count;
        	IMU_buffer[14]=gps_payload_error_count;
        	IMU_buffer[15]=gps_checksum_error_count;
        	IMU_buffer[16]=gps_pos_fix_count;
        	IMU_buffer[17]=gps_nav_fix_count;
        	IMU_buffer[18]=gps_messages_sent;

      	for (int i=0;i<ck+2;i++) Serial.print (IMU_buffer[i]);  
      	for (int i=0;i<ck+2;i++) {
          		IMU_ck_a+=IMU_buffer[i];  //Calculates checksums
          		IMU_ck_b+=IMU_ck_a;       
      	}
      	Serial.print(IMU_ck_a);
      	Serial.print(IMU_ck_b);
      
#else


    Serial.print("PPP");
    Serial.print("pTm:");
    Serial.print(time,DEC);
    Serial.print(",mLc:");
    Serial.print(mainLoop_count,DEC);
    Serial.print(",DtM:");
    Serial.print(G_Dt_max,DEC);
    Serial.print(",gsc:");
    Serial.print(gyro_sat_count,DEC);
    Serial.print(",adc:");
    Serial.print(adc_constraints,DEC);
    Serial.print(",rsc:");
    Serial.print(renorm_sqrt_count,DEC);
    Serial.print(",rbc:");
    Serial.print(renorm_blowup_count,DEC);
    Serial.print(",gpe:");
    Serial.print(gps_payload_error_count,DEC);
    Serial.print(",gce:");
    Serial.print(gps_checksum_error_count,DEC);
    Serial.print(",gpf:");
    Serial.print(gps_pos_fix_count,DEC);
    Serial.print(",gnf:");
    Serial.print(gps_nav_fix_count,DEC);
    Serial.print(",gms:");
    Serial.print(gps_messages_sent,DEC);
    Serial.print(",imu:");
    Serial.print((imu_health>>8),DEC);
    Serial.print(",***");
#endif
		// Reset counters
        mainLoop_count = 0;
        G_Dt_max  = 0;
        gyro_sat_count = 0;
        adc_constraints = 0;
        renorm_sqrt_count = 0;
        renorm_blowup_count = 0;
        gps_payload_error_count = 0;
        gps_checksum_error_count = 0;
        gps_pos_fix_count = 0;
        gps_nav_fix_count = 0;
        gps_messages_sent = 0;
        

}


long convert_to_dec(float x)
{
  return x*10000000;
}

//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  float op=0;
  
  for(int c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar. 
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  for(int c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}



/**************************************************/
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
  float op[3]; 
  for(int x=0; x<3; x++)
  {
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
      {
       op[w]=a[x][w]*b[w][y];
      } 
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];
      
      float test=mat[x][y];
    }
  }
}


extern unsigned long timer0_millis;

// this function replaces the arduino millis() funcion
unsigned long DIYmillis()
{
   unsigned long m;
   unsigned long m2;

   // timer0_millis could change inside timer0 interrupt and we don\u00b4t want to disable interrupts 
   // we can do two readings and compare.
   m = timer0_millis;
   m2 = timer0_millis;
   if (m!=m2)               // timer0_millis corrupted?
      m = timer0_millis;   // this should be fine...
   return m;
}

void DIYdelay(unsigned long ms)
{
   unsigned long start = DIYmillis();
   while (DIYmillis() - start <= ms)
      ;
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

