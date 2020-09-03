#include "Si7021Support.h"
#include "UniGlobals.h"

Si7021::Si7021()
{
}

void Si7021::begin()
{
  //Wire.begin();
  sensor.begin();
}
void Si7021::read(HumidityAnswer& dt)
{
  dt.Humidity = NO_TEMPERATURE_DATA;
  dt.HumidityDecimal = 0;
  dt.Temperature = NO_TEMPERATURE_DATA;
  dt.TemperatureDecimal = 0;

  float humidity, temperature;
  humidity = sensor.readHumidity();
  temperature = sensor.readTemperature();

  if(((int)humidity) == HTU21D_ERROR || ((int)temperature) == HTU21D_ERROR)
  {
    // no data
  }
  else
  {
    // has data
     
    int iTmp = humidity*100;
    
    dt.Humidity = iTmp/100;
    dt.HumidityDecimal = iTmp%100;
    
    iTmp = temperature*100;
    
    dt.Temperature = iTmp/100;
    dt.TemperatureDecimal = abs(iTmp%100);   
  }
}
