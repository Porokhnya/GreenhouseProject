#pragma once

#include <Arduino.h>
//--------------------------------------------------------------------------------------------------------------------------------------
class WeatherStationClass
{
  private:
    int16_t pin;
    static void read_input();
    
  public:
    WeatherStationClass();

    void setup(int16_t pin);
    void update();

  int8_t Humidity; // целое значение влажности
  uint8_t HumidityDecimal; // значение влажности после запятой
  int8_t Temperature; // целое значение температуры
  uint8_t TemperatureDecimal; // значение температуры после запятой
    

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern WeatherStationClass WeatherStation;
