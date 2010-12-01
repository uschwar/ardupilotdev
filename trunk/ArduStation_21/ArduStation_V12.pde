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
  lcd.print("Bonjour JL Naudin");//Writing welcome on the LCD
  Serial.print("Bonjour JL Naudin");//Sending the same by serial port
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
