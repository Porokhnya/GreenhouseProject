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

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern WeatherStationClass WeatherStation;
