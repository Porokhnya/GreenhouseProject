#ifndef _LCD_MODULE_H
#define _LCD_MODULE_H

#include "AbstractModule.h"
#include "Encoder.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class LCDModule : public AbstractModule // модуль поддержки экрана 128х64 на чипе ST7920
{
  private:

  uint16_t waitInitCounter;
  bool inited;
  
  public:
    LCDModule() : AbstractModule("LCD") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LCD_MODULE
extern RotaryEncoder rotaryEncoder;
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
