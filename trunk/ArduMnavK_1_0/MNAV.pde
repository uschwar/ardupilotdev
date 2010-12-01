// Crossbow µNav interface datalink for ardupilot
// october 20, 2010 by Jean-Louis Naudin
//
// ----------------------------------------------------------------------------

byte MNAV_buffer[30];
byte payload_length	= 0;
byte payload_counter	= 0;

void read_Mnav()
{	static unsigned long MNAV_timer = 0; //used to set PROBLEM flag if no data is received. 
	static byte MNAV_step = 0;
	int numc = 0;
	byte data;
	static byte message_num = 0;

	payload_counter = 0;

	numc = Serial.available();
	if (numc > 0){
		for (int i=0;i<numc;i++){	// Process bytes received
			data = Serial.read();

                        // Serial.print(" ");    Serial.print(data,HEX);
                         
			switch(MNAV_step){	 	//Normally we start from zero. This is a state machine

			case 0:	
				if(data == 0x55){  //  'U'
					MNAV_step++; //First byte of data packet header is correct, so jump to the next step
				}
				break; 

			case 1:	
				if(data == 0x55){  // 'U'
					 MNAV_step++;	//Second byte of data packet header is correct
                                  }else {	
					MNAV_step=0;		 //Second byte is not correct so restart to step zero and try again.	
				}	 
				break;

			case 2:	
				if(data == 0x73){ // 's'  Scaled mode output
					 MNAV_step++;	//Third byte of data packet header is correct
				}else {
					MNAV_step=0;		 //Third byte is not correct so restart to step zero and try again.
				}		 
				break;
		 
			case 3:	// Payload data read...
				// We stay in this state until we reach the payload_length
				MNAV_buffer[payload_counter] = data;

				payload_counter++;
				if (payload_counter > 30){
  					payload_counter=0;
					MNAV_step++; 
				}
				break;
			case 4:
					 // Variable initialization
				MNAV_step = 0;
				payload_counter = 0;
                               // Mnavdata_debug();
                                Mnavdata_Parse();
				MNAV_timer = DIYmillis(); //Restarting timer...
				break;

		        }

	}// End for...

	if((DIYmillis() - MNAV_timer) > 500){	//If we don't receive MNAV data in 1/2 second, set flag
		MNAV_ok = false;
	}
  }
}

void Kalman_result()
{ if(MNAV_ok==1 )
   {
   Serial.print("Roll:");       Serial.print(angleR,DEC);
   Serial.print("\tPitch:");    Serial.print(angleN,DEC);
   /*
   Serial.print("\t\tax:");     Serial.print(ax,DEC);
   Serial.print("\tay:");       Serial.print(ay,DEC);
   Serial.print("\taz:");       Serial.print(az,DEC);
   Serial.println();
   Serial.print("hx:");     Serial.print(hx,DEC);
   Serial.print("\thy:");   Serial.print(hy,DEC);
   Serial.print("\thz:");   Serial.print(hz,DEC);
   */
   Serial.println();
   }
}

void Mnavdata_Out()
{  if(MNAV_ok==1 )
   {
   Serial.print("ax:");     Serial.print(ax,DEC);
   Serial.print("\tay:");   Serial.print(ay,DEC);
   Serial.print("\taz:");   Serial.print(az,DEC);
   Serial.print("\t\tp:");  Serial.print(p,DEC);
   Serial.print("\tq:");    Serial.print(q,DEC);
   Serial.print("\tr:");    Serial.println(r,DEC);
   Serial.print("hx:");     Serial.print(hx,DEC);
   Serial.print("\thy:");   Serial.print(hy,DEC);
   Serial.print("\thz:");   Serial.print(hz,DEC);
   Serial.print("\tPs:");   Serial.print(Ps,DEC);
   Serial.print("\tPt:");   Serial.println(Pt,DEC);
   }

}

void Mnavdata_Parse()
{  int j=0, mcksum=0;
   signed short tmp=0;
   
   mcksum=checksum(MNAV_buffer,30);
   if (checksum(MNAV_buffer,30))
   {
  	 //Storing IMU Accel X in m/s^2

    ax=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*5.98755e-04; tmp=0;
    
  	 //Storing IMU Accel Y in m/s^2
    
    ay=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*5.98755e-04; tmp=0;
    
      	 //Storing IMU Accel Z in m/s^2
    
    az=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*5.98755e-04; tmp=0;
    
      	 //Storing IMU angular rate X in rad/s

    p=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*1.06526e-04; tmp=0;
    
  	 //Storing IMU angular rate Y in rad/s
    
    q=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*1.06526e-04; tmp=0;    
    
      	 //Storing IMU angular rate Z in rad/s
    
    r=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*1.06526e-04; tmp=0;   
    
         //Storing IMU magnetic field X in gauss
    
    hx=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*6.10352e-05; tmp=0;
    
         //Storing IMU magnetic field Y in gauss
    
    hy=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*6.10352e-05; tmp=0;
    
         //Storing IMU magnetic field Z in gauss
    
    hz=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*6.10352e-05; tmp=0;
    
         //Storing IMU pressure (alt) Ps in meter
    
    Ps=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*3.05176e-01; tmp=0;
    
         //Storing IMU pressure (speed) Pt in m/s
    
    Pt=(double)(((tmp=MNAV_buffer[j++])<<8)|MNAV_buffer[j++])*2.44141e-03; tmp=0;
    
   // Mnavdata_Out();
   }
    
}

void Mnavdata_debug()
{     
  Serial.print("UUV ->");
  for (int i=0;i<30;i++){ 
      Serial.print(MNAV_buffer[i],HEX);
      Serial.print(" ");
  }
    Serial.println("");
}

void wait_for_data(byte many)
{
	while(Serial.available() <= many); 
}

/***************************************************************************************
 *check the checksum of the data packet
 ***************************************************************************************/
int checksum(byte* buffer, int packet_len)
{
   word     	 i=0,rcvchecksum=0;
   //unsigned long sum=0;
   word          sum=0;
    
   sum = 's';
   for(i=0;i<packet_len-2;i++) sum = sum + buffer[i];
   rcvchecksum = ((rcvchecksum = buffer[packet_len-2]) << 8) | buffer[packet_len-1];
   
  //Serial.print("CKSUM = ");  Serial.print(rcvchecksum,DEC);
  // Serial.print("\tREC = ");  Serial.println(sum,DEC);
   
// if (rcvchecksum == sum%0x10000)
   if (rcvchecksum == sum) //&0xFFFF)
     {   MNAV_ok=1;
	return    1;
     }
   else
      { MNAV_ok=0;
 	return    0;
      }
}

/*
void write_package_to_uart0( uint8_t *buf , uint8_t package_type, uint8_t size )
{
	uint16_t	checksum;
	uint8_t		i;
	putc( 0x55 );
	putc( 0x55 );
	putc( package_type );
	checksum = package_type;
	for( i = 0 ; i < size ; i++ )
	{
		putc( *( buf + i ) );
		checksum += *( buf  + i );
	}
	putc( ( checksum >> 8 ) & 0xFF );
	putc( ( checksum >> 0 ) & 0xFF );
}

int get_msg(struct imugpsio* io, byte* input_buffer)
{
   struct circbuf *c = &io->buf;
   int count = 0;
   int packet_len;
   int i;
  
   read_into_buffer(io);

   while (1) {

       //*Find start of packet: the heade r (2 bytes) starts with 0x5555

      while(c->circbufLength >= 4 && (c->circbuf[c->circbufStart] != (byte)0x55 || c->circbuf[(byte)(c->circbufStart + 1)] != (byte)0x55)) {
         c->circbufStart++;
         c->circbufLength--;
      }
      if(c->circbufLength < 4)
         return count;

       //*Read packet contents
       
      packet_len = 0;
      switch (c->circbuf[(byte)(c->circbufStart + 2)])
      {
		case 'S':  
			  packet_len = SENSOR_PACKET_LENGTH;
			  break;
    
		case 'N': 
			  packet_len = FULL_PACKET_SIZE;
			  break;
    
		case 'V':  
			  printf("[imu]:received voltage packet!\n");
			  break;
    
		default:
			  break;
      }
  
      if(packet_len > 0 && c->circbufLength < packet_len)
         return count;
      if(packet_len > 0) {
         byte ib;
         word rcvchecksum = 0;
         word sum = 0;
      
         for(i = 2, ib = c->circbufStart + (byte)2; i < packet_len - 2; i++, ib++)
            sum += c->circbuf[ib];
         rcvchecksum = c->circbuf[ib++] << 8;
         rcvchecksum = rcvchecksum | c->circbuf[ib++];
      
         if(rcvchecksum != sum)
            packet_len = 0;
      }
      if(packet_len > 0) {
         for(i = 0; i < packet_len; i++) {
            input_buffer[i] = c->circbuf[c->circbufStart];
            c->circbufStart++;
            c->circbufLength--;
         }
         count++;
      }
      else {
         c->circbufStart += 3;
         c->circbufLength -= 3;
      }
  
  
   } 
  
   return count;
}

*/



