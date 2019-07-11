#include "Si7021Support.h"
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
Si7021::Si7021()
{
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Si7021::begin()
{
  sensor.begin();
}
//--------------------------------------------------------------------------------------------------------------------------------------
HumidityAnswer Si7021::read()
{

  HumidityAnswer dt;
  dt.IsOK = false;
  dt.Humidity = NO_TEMPERATURE_DATA;
  dt.Temperature = NO_TEMPERATURE_DATA;
  
  float humidity, temperature;
  humidity = sensor.readHumidity();
  temperature = sensor.readTemperature();
  
  
/*
  byte humError = (byte) humidity;
  byte tempError = (byte) temperature;
  if(humError == HTU21D_ERROR || tempError == HTU21D_ERROR)
  {
    dt.IsOK = false;
  }
  else
  {
*/  
     dt.IsOK = true;
     int iTmp;

    if(humidity > -1.0)
    { 
      iTmp = humidity*100;
      
      dt.Humidity = iTmp/100;
      dt.HumidityDecimal = iTmp%100;
  
      if(dt.Humidity < 0 || dt.Humidity > 100)
      {
        dt.Humidity = NO_TEMPERATURE_DATA;
        dt.HumidityDecimal = 0;
      }
    }

    if(temperature > -200.0)
    {      
        iTmp = temperature*100;
    
        int8_t t1 =     iTmp/100;
        uint8_t t2 = abs(iTmp%100);
        
        dt.Temperature = t1;
        dt.TemperatureDecimal = t2;
    
        if(dt.Temperature < -40 || dt.Temperature > 125)
        {
          dt.Temperature = NO_TEMPERATURE_DATA;
          dt.TemperatureDecimal = 0;
        }
    }
       
/////////////////////  } 

  return dt;
}
//--------------------------------------------------------------------------------------------------------------------------------------
