#ifndef _SI7021_SUPPORT_H
#define _SI7021_SUPPORT_H

#include <Arduino.h>
#include <Wire.h>
#include "HumidityGlobals.h"
#include "Adafruit_Si7021.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class Si7021
{
  public:
  
    Si7021();    
    void begin();
    
    HumidityAnswer read();
    
  private:
    Adafruit_Si7021 sensor;

    
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
