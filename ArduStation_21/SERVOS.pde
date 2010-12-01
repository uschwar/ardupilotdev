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
