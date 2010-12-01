
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
