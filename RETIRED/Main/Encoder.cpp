#include "Encoder.h"
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
RotaryEncoder::RotaryEncoder(byte p0, byte p1, byte pulsesPerClick)
{
  pin0 = p0;
  pin1 = p1;
  ppc = pulsesPerClick;
  change = 0;
  state = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RotaryEncoder::begin()
{
  WORK_STATUS.PinMode(pin0, INPUT);
  WORK_STATUS.PinMode(pin1, INPUT);
  digitalWrite(pin0, HIGH);   // включаем подтягивающие резисторы
  digitalWrite(pin1, HIGH);
  change = 0;
  delay(10);                  // читаем состояние после небольшого ожидания
  state = readState();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RotaryEncoder::update()
{
  // State transition table. Each entry has the following meaning:
  // 0 - the encoder hasn't moved
  // 1 - the encoder has moved 1 unit clockwise
  // -1 = the encoder has moved 1 unit anticlockwise
  // 2 = illegal transition, we must have missed a state
  static int tbl[16] = 
  {  0, +1, -1, 0,    // position 3 = 00 to 11, can't really do anything, so 0
    -1,  0, -2, +1,   // position 2 = 01 to 10, assume it was a bounce and should be 01 -> 00 -> 10  
    +1, +2,  0, -1,   // position 1 = 10 to 01, assume it was a bounce and should be 10 -> 00 -> 01
     0, -1, +1, 0     // position 0 = 11 to 10, can't really do anything
  };
  unsigned int t = readState();
  int movement = tbl[(state << 2) | t];
  if (movement != 0)
  {
    change += movement;
    state = t; 
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
unsigned int RotaryEncoder::readState()
{
  return (digitalRead(pin0) ? 1u : 0u) | (digitalRead(pin1) ? 2u : 0u);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int RotaryEncoder::getChange()
{
  int r;
  noInterrupts();
  if (change >= ppc - 1)
  {
    r = (change + 1)/ppc;
  }
  else if (change <= 1 - ppc)
  {
    r = -((1 - change)/ppc);
  }
  else
  {
    r = 0;
  }
  change -= (r * ppc);
  interrupts();
  return r;
}
//--------------------------------------------------------------------------------------------------------------------------------------


