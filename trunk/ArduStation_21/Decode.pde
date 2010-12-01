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

