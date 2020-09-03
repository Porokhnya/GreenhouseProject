#include "DS18B20Query.h"
#include "CONFIG.h"
#include <OneWire.h>
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Support::begin(uint8_t _pin) 
{
  pin = _pin;
  pinMode(pin,INPUT);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Support::setResolution(DS18B20Resolution res)
{
  if(!pin)
    return;

  OneWire ow(pin);
 

  if(!ow.reset()) // нет датчика
    return;  

   ow.write(0xCC); // пофиг на адреса (SKIP ROM)
   ow.write(0x4E); // запускаем запись в scratchpad

   ow.write(0); // верхний температурный порог 
   ow.write(0); // нижний температурный порог
   ow.write(res); // разрешение датчика

   ow.reset();
   ow.write(0xCC); // пофиг на адреса (SKIP ROM)
   ow.write(0x48); // COPY SCRATCHPAD
   delay(10);
   ow.reset();
   
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Support::readTemperature(DS18B20Temperature* result,DSSensorType type)
{
  result->Whole = NO_TEMPERATURE_DATA; // нет данных с датчика
  result->Fract = 0;

  if(!pin)
    return false;


  OneWire ow(pin);

  if(!ow.reset()) // нет датчика
    return false;

  byte data[9];
   
  ow.write(0xCC); // пофиг на адреса (SKIP ROM)
  ow.write(0x44); // запускаем преобразование

  ow.reset();
  ow.write(0xCC); // пофиг на адреса (SKIP ROM)
  ow.write(0xBE); // читаем scratchpad датчика на пине

  for(uint8_t i=0;i<9;i++)
    data[i] = ow.read();


 if (OneWire::crc8( data, 8) != data[8]) // проверяем контрольную сумму
      return false;
  
  int loByte = data[0];
  int hiByte = data[1];

  int temp = (hiByte << 8) + loByte;
  
  result->Negative = (temp & 0x8000);
  
  if(result->Negative)
    temp = (temp ^ 0xFFFF) + 1;

  int tc_100 = 0;
  switch(type)
  {
    case DS18B20:
      tc_100 = (6 * temp) + temp/4;
    break;

    case DS18S20:
      tc_100 = (temp*100)/2;
    break;
  }
   

  result->Whole = tc_100/100;
  result->Fract = abs(tc_100 % 100);

  if(result->Whole < -55 || result->Whole > 125)
  {
    result->Negative = false;
    result->Whole = NO_TEMPERATURE_DATA;
    result->Fract = 0;
  }

  return true;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------

