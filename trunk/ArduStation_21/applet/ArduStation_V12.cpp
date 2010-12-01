#include <LiquidCrystal.h>
#include <avr/eeprom.h>

//Arduino Develoment Board Demo (code name: ArduStation)
//Version 1.2
//Released under Creative Commons License. 
//By Jordi Munoz
//Version Updated by Jean-Louis Naudin - 11-14-09

#define ToRad(x) (x*PI)/180.0
#define ToDeg(x) (x*180.0)/PI


#define SERVO_MAX 2000 //Range of servos pitch and roll
#define SERVO_MIN 900

#include "WProgram.h"
void setup();
void loop();
byte NewDataFlag(void);
void SetDataFlag(void);
unsigned char Button_0();
unsigned char Button_1();
unsigned char Button_2();
unsigned char Button_3();
unsigned char Button_4();
unsigned char Button_5();
byte Lock_Button(byte button, byte inLock[1], int time);
void buzz(int time, int freq);
void decode(void);
float Search_Float(char buffer[],char looking[]);
int Search_Int(char buffer[],char looking[]);
long Search_Long(char buffer[],char looking[]);
float calc_bearing(float flat1, float flon1, float flat2, float flon2);
float calc_dist(float flat1, float flon1, float flat2, float flon2);
float to_float_6(long value);
void Check_Buttons(byte max_options);
void Init_servos(void);
void pulse_servo_1(long angle);
void pulse_servo_2(long angle);
void sub_actions(void);
void RestoreData(void);
void Compass(int heading);
char *BufferPointer; //Pointers
char *SearchString;

byte data_update_event=1;//New data flag 

//Incoming variables. 
int airSpeed=0;
int throttle=0;
int roll=0;
int pitch=0;

long latitude=0;
long longitud=0; 
int altitude=0;
int holdAltitude=0;
int climbRate=0;
int groundSpeed=0;


int course=0;
int bearing=0;
int waypoint=0;
long distance=1;
float battery=0;
int roll_set_point=0;

//Menu variables
byte menu=1;
byte submenu[5];


long Latitude_Home=0;
long Longitud_Home=0;
float Distance_Home=0;
float Distance3D_Home=0;
int Angle_Home=0;
float Bearing_Home=0;

// LiquidCrystal display with:
// rs on pin 2
// rw on pin 3
// enable on pin 4
// d4, d5, d6, d7 on pins 5, 6, 7, 8
LiquidCrystal lcd(2, 3, 4, 5, 6, 7, 8); //Initailizing LCD object (this is real C++). 

uint8_t  N[8] = {0x00,0x04,0x0E,0x15,0x04,0x04,0x00,0x00};
uint8_t NE[8] = {0x00,0x07,0x03,0x05,0x08,0x10,0x00,0x00};
uint8_t  E[8] = {0x00,0x04,0x02,0x1F,0x02,0x04,0x00,0x00};
uint8_t SEE[8] ={0x00,0x10,0x08,0x05,0x03,0x07,0x00,0x00};
uint8_t  S[8] = {0x00,0x04,0x04,0x15,0x0E,0x04,0x00,0x00};

uint8_t  SW[8] ={0x00,0x01,0x02,0x14,0x18,0x1C,0x00,0x00};
uint8_t  W[8] = {0x00,0x04,0x08,0x1F,0x08,0x04,0x00,0x00};
uint8_t  NW[8] ={0x00,0x1C,0x18,0x14,0x02,0x01,0x00,0x00};

void setup()
{
  RestoreData();
  lcd.begin(20, 4);
  lcd.createChar(0, N);
  lcd.createChar(1, NE);
  lcd.createChar(2, E);
  lcd.createChar(3, SEE);
  lcd.createChar(4, S);
  lcd.createChar(5, SW);
  lcd.createChar(6, W);
  lcd.createChar(7, NW);
  
  Serial.begin(38400); //Serial  
  pinMode(14,OUTPUT); //Pin mode as output to control buzzer (analog0)
  lcd.clear(); //Clearing screen
  lcd.print("Bonjour Jean-Louis Naudin");//Writing welcome on the LCD
  Serial.print("Bonjour !");//Seding the same by serial port
  //lcd.setCursor(collum, row);
  lcd.setCursor(0, 1);//Setting cursor to col 0, row 1
  lcd.print("Station sol portable ");
  lcd.setCursor(0, 2);
  lcd.print("  Nov 2009  ");
  lcd.setCursor(6, 3);
  lcd.print(0,BYTE);
  lcd.print(1,BYTE);
  lcd.print(2,BYTE);
  lcd.print(3,BYTE);
  lcd.print(4,BYTE);
  lcd.print(5,BYTE);
  lcd.print(6,BYTE);
  lcd.print(7,BYTE);
  buzz(500,200); //Pulsing the buzzer
  delay(1500); 
  pinMode(11,OUTPUT);//??? i don't remember :S
  lcd.setCursor(0, 2);
}

void loop()
{
  Check_Buttons(4);
  decode(); //Decoding data comming from ardupilot

  if(NewDataFlag()) //Checking if new data arrived or we've pressed a button. 
  {  
    switch(menu) //Menu to display
    {
    case 1:
      lcd.clear(); //Printing all the stuff 
      lcd.print("WP#~");
      lcd.print(waypoint);
      lcd.setCursor(10, 0);
      lcd.print("DIS~");
      lcd.print(distance);
      lcd.setCursor(0, 1);
      lcd.print("CRS~");
      lcd.print(course);
      lcd.setCursor(8, 1);
      Compass(course);
      lcd.setCursor(10, 1);
      lcd.print("BER~");
      lcd.print(bearing);
      lcd.setCursor(18, 1);
      Compass(bearing);
      lcd.setCursor(0, 2);
      lcd.print("GSP~");
      lcd.print(groundSpeed);
      lcd.setCursor(10, 2);
      lcd.print("ASP~");
      lcd.print(airSpeed);
      lcd.setCursor(0, 3);
      lcd.print("ALT~");
      lcd.print(altitude);
      lcd.setCursor(10, 3);
      lcd.print("Batt~");
      lcd.print(battery);

      break;
    case 2:
      lcd.clear();
      lcd.print("Roll~");
      lcd.print(roll);
      lcd.setCursor(10, 0);
      lcd.print("Pitch~");
      lcd.print(pitch);
      lcd.setCursor(0, 1);
      lcd.print("ASP~");
      lcd.print(airSpeed);
      lcd.setCursor(10, 1);
      lcd.print("GSP~");
      lcd.print(groundSpeed);
      lcd.setCursor(0, 2);
      lcd.print("ALT~");
      lcd.print(altitude);
      lcd.setCursor(10, 2);
      lcd.print("DAL~");
      lcd.print(holdAltitude);
      lcd.setCursor(0, 3);
      lcd.print("CRS~");
      lcd.print(course);
      lcd.setCursor(10, 3);
      lcd.print("RSP~");
      lcd.print(roll_set_point);    
      break;
      
    case 3:
      Distance_Home=calc_dist(to_float_6(Latitude_Home), to_float_6(Longitud_Home), to_float_6(latitude), to_float_6(longitud));
      Distance3D_Home= sqrt((Distance_Home*Distance_Home)+(altitude*altitude));
      Angle_Home=ToDeg(atan((float)altitude/(float)Distance_Home));
      Bearing_Home=calc_bearing(to_float_6(Latitude_Home), to_float_6(Longitud_Home), to_float_6(latitude), to_float_6(longitud));
      lcd.clear();
      lcd.print("Dist2D~");
      lcd.print(Distance_Home);
      lcd.setCursor(0, 1);
      lcd.print("Dist3D~");
      lcd.print(Distance3D_Home);
      lcd.setCursor(0, 2);
      lcd.print("Angle~");
      lcd.print(Angle_Home);
      lcd.setCursor(10, 2);
      lcd.print(" Alt~");
      lcd.print(altitude);
      lcd.setCursor(0, 3);
      lcd.print("Dir~");
      lcd.print(Bearing_Home);
      
    break;
    case 4:
      lcd.clear(); //Printing all the stuff 
      lcd.print("Set Home?");
      lcd.setCursor(0, 1); 
      lcd.print(latitude);
      lcd.setCursor(10, 1);
      lcd.print(longitud);
      lcd.setCursor(0, 2);
      lcd.print("Press OK to Save");
      sub_actions();
      lcd.setCursor(0, 3);
      lcd.print(Latitude_Home);
      lcd.setCursor(10, 3);
      lcd.print(Longitud_Home);
    break;
    default:

      break;
    }
  }
  
//sub_actions();



}


byte NewDataFlag(void)
{
  if((data_update_event&0x01)==0x01)
  {
    data_update_event&=0xFE;
    return 1;
  }
  else
  {
    return 0; 
  }

}

void SetDataFlag(void)
{
  data_update_event|=0x01; 
}

//i had to make a miracle here in order to read 6 buttons in 3 pins. =)
//You better read the schematic of the Board in order to understand this. 
unsigned char Button_0()
{
  static byte lock; 
  pinMode(13,LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  pinMode(12, INPUT);

  if(digitalRead(12) == HIGH)
  return 1;
  else
  return 0;
}

//Button2: PORTB6 output, check PINB7
unsigned char Button_1()
{
  static byte lock;
  digitalWrite(11, LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  pinMode(13, INPUT);

  if(digitalRead(13) == HIGH)
  return 1;
  else
  return 0;
}

//Button3: PORTB7 output, check PINB5
unsigned char Button_2()
{
  static byte lock;
  pinMode(12,LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(11, INPUT);
  if(digitalRead(11) == HIGH)
  return 1;
  else
  return 0;
}

//Button4: PORTB6 output, check PINB5
unsigned char Button_3()
{
  static byte lock;
  pinMode(13,LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  pinMode(11, INPUT);
  if(digitalRead(11) == HIGH)
  return 1;
  else
  return 0;
}

//Button5: PORTB7 output, check PINB6
unsigned char Button_4()
{
  static byte lock;
  pinMode(11,LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(12, INPUT);
  if(digitalRead(12) == HIGH)
  return 1;
  else
  return 0;
}

//Button1: PORTB6 output, check PINB7
unsigned char Button_5()
{
  pinMode(12,LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  pinMode(13, INPUT);
  
  if(digitalRead(13)==HIGH)
  return 1;
  else
  return 0;
}

/*********************************************/

/*********************************************/

/*********************************************/

byte Lock_Button(byte button, byte inLock[1], int time)
{
    if(button == 1)
  {
    if(inLock[0] == 0)
    {
      inLock[0]=1;
      buzz(time,200);
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    inLock[0]=0;
    return 0; 
  }
}


void buzz(int time, int freq)
{
  cli();
  for(int c=0; c<time; c++)
  {
    digitalWrite(14,HIGH); 
    delayMicroseconds(freq);
    digitalWrite(14,LOW); 
    delayMicroseconds(freq);
  }
  sei();
}
void decode(void) //Yeah i know, after a week of made this i was unable to understand it. 
{
  static char buffer[250]; //The traditional SERIAL buffer.
  static int counter=0; //Traditional counter

  while(Serial.available() > 0) //Check and loop while any byte is in the Serial Buffer. 
  {

    buffer[counter]=Serial.read();//Reading the byte

    if(buffer[counter]==0x0A)//Looks for \F, if not jump and continue receveing. 
    {

      if (strncmp (buffer,"!!!",3) == 0) //Looks for the !!! header
      {
        latitude=Search_Long(buffer,"LAT:");//Searching for the string LAT:, and converting/storing the value next to it. 
        longitud=Search_Long(buffer,"LON:"); //The same
        altitude=Search_Int(buffer,"ALT:"); //The same but convert it to intrigers 
        holdAltitude=Search_Int(buffer,"ALH:");
        climbRate=Search_Int(buffer,"CRT:");
        course=Search_Int(buffer,"CRS:");
        bearing=Search_Int(buffer,"BER:");
        groundSpeed=Search_Int(buffer,"SPD:");
        waypoint=Search_Int(buffer,"WPN:");
        distance=Search_Long(buffer,"DST:");
        battery=Search_Float(buffer,"BTV:");//The same but convert it to floats!
        roll_set_point=Search_Int(buffer,"RSP:");

        data_update_event|=0x01; //This is a manual flag to indicate new data has arrived.
      }
      if (strncmp (buffer,"+++",3) == 0) //Looks for the +++
      {
        airSpeed=Search_Float(buffer,"ASP:"); //Searching for the string ASP:, and converting/storing the value next to it. 
        roll=Search_Int(buffer,"RLL:");
        pitch=Search_Int(buffer,"PCH:");
        throttle==Search_Int(buffer,"TTH:");
        data_update_event|=0x01; //This is a manual flag to indicate new data has arrived. 
      }
       
      for(int a=0; a<=counter; a++)//restarting the buffer
      {
        buffer[a]=0; //Clearing the buffer.
      } 
      counter=0; //Restarting the counter. 
    }
    else
    {
      counter++;//Incrementing the counter
    }
    
    if(counter >= 250)
    {
       for(int a=0; a<=counter; a++)//restarting the buffer
      {
        buffer[a]=0; //Clearing the buffer.
      } 
      counter=0; //Restarting the counter. 
      break;
    }
  }
  //  

}




//Very important functions:
float Search_Float(char buffer[],char looking[])
{
  char *BufferPointer=buffer;
  char *SearchString;
  SearchString= strstr (BufferPointer,looking); 
  return atof(SearchString+4);
}

int Search_Int(char buffer[],char looking[])
{
  char *BufferPointer=buffer;
  char *SearchString;
  SearchString= strstr (BufferPointer,looking); 
  return atoi(SearchString+4);
}

long Search_Long(char buffer[],char looking[])
{
  char *BufferPointer=buffer;
  char *SearchString;
  SearchString= strstr (BufferPointer,looking); 
  return atol(SearchString+4);
}

/*************************************************************************
 * //Function to calculate the course between two waypoints
 * //I'm using the real formulas--no lookup table fakes!
 *************************************************************************/
float calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
  calc=atan2(y,x);

  bear_calc= degrees(calc);

  //if(bear_calc<=1){
    //bear_calc=360+bear_calc; 
  //}
  return bear_calc;
}
/*************************************************************************
 * //Function to calculate the distance between two waypoints
 * //I'm using  a really good approach
 *************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
 float x = 69.1 * (flat2 - flat1); 
 float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
 return (float)sqrt((float)(x*x) + (float)(y*y))*1609.344; 
}

float to_float_6(long value) 
{
  return (float)value/(float)1000000;
}
void Check_Buttons(byte max_options) //Reading the buttons. 
{
  static byte lock[6];
  byte button_byte=0;

  if(Lock_Button(Button_0(), &lock[0], 200))
  {
    SetDataFlag();
  }

  if(Lock_Button(Button_1(), &lock[1], 200))
  {
    SetDataFlag();
  }

  if(Lock_Button(Button_2(), &lock[2], 200))
  {
    SetDataFlag();
    menu--;
  }

  if(Lock_Button(Button_3(), &lock[3], 200))
  {
    SetDataFlag();
  }

  if(Lock_Button(Button_4(), &lock[4], 200))
  {
    SetDataFlag();
    menu++;
  }

  if(Lock_Button(Button_5(), &lock[5], 200))
  {
    SetDataFlag();
  } 

  if(menu<1)
    menu=max_options;

  if(menu>max_options)
  {
    menu=1;
  }
}
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
  sei();//Enabling interrupts
}


/**************************************************************
 * Function to pulse servo 1
 ***************************************************************/
void pulse_servo_1(long angle)//Will convert the angle to the equivalent servo position... 
{
  angle=constrain(angle,0,180);
  OCR1A=((angle*(SERVO_MAX-SERVO_MIN))/180L+SERVO_MIN)*2L;
}

/**************************************************************
 * Function to pulse the servo 2... 
 ***************************************************************/
void pulse_servo_2(long angle)//Will convert the angle to the equivalent servo position... 
{
  angle=constrain(angle,0,180);
  OCR1B=((angle*(SERVO_MAX-SERVO_MIN))/180L+SERVO_MIN)*2L; //Scaling
}
void sub_actions(void)
{
  static byte lock[6];
switch(menu) //Menu to display
    {
    case 1:

      break;
    case 2:

      break;
    case 4:
    
      if(Button_3())
      {
        Latitude_Home=latitude;
        Longitud_Home=longitud;
        lcd.setCursor(0, 2);
        lcd.print("Done!!              ");
          eeprom_busy_wait();
          eeprom_write_dword((unsigned long*)0x00,(long)(Latitude_Home));
          eeprom_busy_wait();
          eeprom_write_dword((unsigned long*)0x04,(long)(Longitud_Home));
      }
    break;
    default:

      break;
    }
    
}

void RestoreData(void)
{
  eeprom_busy_wait();
  Latitude_Home=(long)eeprom_read_dword((unsigned long*)0x00);
  eeprom_busy_wait();
  Longitud_Home=(long)eeprom_read_dword((unsigned long*)0x04);
}
  
  void Compass(int heading)
  {
    if(((heading>=338)&&(heading<=360))||((heading>=-22)&&(heading<=22)))
    {
      lcd.print(0,BYTE);
    }
    if((heading>=23)&&(heading<=68))
    {
      lcd.print(1,BYTE);
    }
     if((heading>=69)&&(heading<=113))
    {
      lcd.print(2,BYTE);
    }
    if((heading>=114)&&(heading<=158))
    {
      lcd.print(3,BYTE);
    }    
    if((heading>=159)&&(heading<=202))
    {
      lcd.print(4,BYTE);
    }   
    if((heading>=203)&&(heading<=248))
    {
      lcd.print(5,BYTE);
    }
    if((heading>=249)&&(heading<=292))
    {
      lcd.print(6,BYTE);
    }      
    if((heading>=293)&&(heading<=337))
    {
      lcd.print(7,BYTE);
    }      
  }

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

