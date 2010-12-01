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
