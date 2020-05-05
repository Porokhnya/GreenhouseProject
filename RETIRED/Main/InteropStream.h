#ifndef _INTEROP_STREAM_H
#define _INTEROP_STREAM_H

#include <Arduino.h>
#include <WString.h>
#include "ModuleController.h"
#include "CommandParser.h"
//--------------------------------------------------------------------------------------------------------------------------------
// класс поддержки коммуникации между модулями
//--------------------------------------------------------------------------------------------------------------------------------
class CommandExecuteResult : public Stream
{
  public:

  String buffer;
  
   // Stream
  virtual void flush(){}
  virtual int peek() {return 0;}
  virtual int read() {return 0;}
  virtual int available() {return 0;}
  virtual size_t write(uint8_t ch) { buffer += (char) ch; return 1;}  
  
}; 
//--------------------------------------------------------------------------------------------------------------------------------
class InteropStream
{
private:

public:

   InteropStream();
  ~InteropStream();

    bool QueryCommand(COMMAND_TYPE cType, const String& command, bool isInternalCommand); // вызывает команду для зарегистрированного модуля
  
};
//--------------------------------------------------------------------------------------------------------------------------------
extern InteropStream ModuleInterop;
//--------------------------------------------------------------------------------------------------------------------------------
// класс-хелпер мигания информационным диодом
//--------------------------------------------------------------------------------------------------------------------------------
class BlinkModeInterop
{
  private:

  uint16_t blinkInterval;
  uint16_t timer;
  uint8_t pin;
  uint8_t pinState;
  
  public:
    BlinkModeInterop();

    void begin(uint8_t pin); // запоминаем настройки
    void blink(uint16_t interval=0); // мигаем диодом
    void update(uint16_t dt); // обновляем состояние
};
//--------------------------------------------------------------------------------------------------------------------------------
#endif

