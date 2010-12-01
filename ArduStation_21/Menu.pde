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
