// October 10, 2010 - updated version from r1496 by Jean-Louis Naudin
// Updated on 10-10-10 JLN - corrected bug of the EM406 GPS which unlock each 3 sec with the Shield v2 and the thermopile setup

#if GPS_PROTOCOL == 1

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
  pinMode(12, OUTPUT);//Status led
  delay(500);
  change_to_sirf_protocol();
  Serial.begin(57600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  delay(500);//Waits fot the GPS to start_UP
  configure_gps();//Function to configure GPS, to output only the desired msg's
}

void decode_gps(void)
{
  static unsigned long GPS_timer=0;
  static byte gps_counter=0; //Another gps counter for the buffer
  static byte GPS_step=0;
  static byte gps_ok=0;//Counter to verify the reciving info
  const byte read_gps_header[]={
    0xA0,0xA2,0x00,0x5B,0x29      };//Used to verify the payload msg header

  if(DIYmillis()-GPS_timer > 200) //Timer to execute the loop every 200 ms only.
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
                GPS_timer=DIYmillis(); //Restarting timer... 
                GPS_join_data(); //Joing the data
  
                //GPS_print_data(); // and print the values... 
              }
            }
            GPS_step=0; //Restarting.... 
            break;
          }
        }
        break;
      }
       GPS_timer=DIYmillis(); //Restarting timer... 
    }
    if(millis() - GPS_timer > 2000){
      digitalWrite(12, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED... 
      GPS_fix = BAD_GPS;
      GPS_timer = DIYmillis();
      Serial.println("no GPS, last 20s");
    }
  }
}

void GPS_join_data(void)
{
    GPS_fix=(int)gps_buffer[0]<<8; //Here i'm joining the status GPS 2 Bytes to 1 Int. 
    GPS_fix|=(int)gps_buffer[1];
/*
    if(GPS_fix == 0) //If the status GPS is equals to cero YEAAHH! turn ON the LED
    {
      digitalWrite(12,HIGH);
    }
    else{
      digitalWrite(12,LOW);
    }//If not UHH something is really wrong, don't look at me! is GPS fault!
    */
	// Read bytes and combine them with Unions
	// ---------------------------------------
	byte j = 22;
	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];
	current_loc.lat		= longUnion.dword;		//Y = lat * 10,000,000


	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];
	current_loc.lng 	= longUnion.dword;		// X = long * 10,000,000

	j = 34;
	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];	
	current_loc.alt 	= longUnion.dword;		// alt in meters * 100

	j = 39;
	intUnion.byte[1] 	= gps_buffer[j++];
	intUnion.byte[0] 	= gps_buffer[j++];
	ground_speed 		= intUnion.word;		// meters/second * 100
	//ground_speed = (float)intUnion.word;			// Speed in M/S * 100		

	if(ground_speed >= 50){
		//Only updates data if we are really moving... 
		intUnion.byte[1] 	= gps_buffer[j++];
		intUnion.byte[0] 	= gps_buffer[j++];
		ground_course 		= (unsigned int)intUnion.word;		// degrees * 100

	}else{
		//ground_speed 	= 0;
		//ground_course = 0;
	}

	j = 45;
	intUnion.byte[1] 	= gps_buffer[j++];
	intUnion.byte[0] 	= gps_buffer[j++];
	climb_rate 			= intUnion.word;	//m/s * 100


	GPS_new_data = true;

	// We can't navigate until the GPS has output data
	// ------------------------
	if(current_loc.lat == 0){
		GPS_new_data = false;
		GPS_fix = BAD_GPS;
	}
	
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
	Serial.begin(EM406_GPS); //Universal Sincronus Asyncronus Receiveing Transmiting
}

#endif

