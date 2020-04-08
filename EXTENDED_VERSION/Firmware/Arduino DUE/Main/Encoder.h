#ifndef _ENCODER_H
#define _ENCODER_H

#include <Arduino.h>
//--------------------------------------------------------------------------------------------------------------------------------------
class RotaryEncoder
{
  unsigned int state;
  byte pin0, pin1;
  byte ppc;
  int change;

  unsigned int readState();


public:
  RotaryEncoder(byte p0, byte p1, byte pulsesPerClick);

  void begin();
  void update();
  int getChange();
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
