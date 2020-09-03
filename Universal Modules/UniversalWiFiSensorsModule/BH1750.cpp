#include "BH1750.h"
#include "CONFIG.h"
//--------------------------------------------------------------------------------------------------------------------------------------
BH1750Support::BH1750Support()
{

}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::begin(BH1750Address addr, BH1750Mode mode)
{
  deviceAddress = addr;
  writeByte(BH1750PowerOn); // включаем датчик
  ChangeMode(mode); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::ChangeMode(BH1750Mode mode) // смена режима работы
{
   currentMode = mode; // сохраняем текущий режим опроса
   writeByte((uint8_t)currentMode);
  delay(10);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::ChangeAddress(BH1750Address newAddr)
{
  if(newAddr != deviceAddress) // только при смене адреса включаем датчик
  { 
    deviceAddress = newAddr;
    
    writeByte(BH1750PowerOn); // включаем датчик  
    ChangeMode(currentMode); // меняем режим опроса на текущий
  } // if
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::writeByte(uint8_t toWrite) 
{
  Wire.beginTransmission(deviceAddress);
  BH1750_WIRE_WRITE(toWrite);
  Wire.endTransmission();
}
//--------------------------------------------------------------------------------------------------------------------------------------
long BH1750Support::GetCurrentLuminosity() 
{
  long curLuminosity = NO_LUMINOSITY_DATA;

 if(Wire.requestFrom(deviceAddress, 2) == 2)// ждём два байта
 {
  // читаем два байта
  curLuminosity = BH1750_WIRE_READ();
  curLuminosity <<= 8;
  curLuminosity |= BH1750_WIRE_READ();
  curLuminosity = curLuminosity/1.2; // конвертируем в люксы
 }


  return curLuminosity;
}
//--------------------------------------------------------------------------------------------------------------------------------------

