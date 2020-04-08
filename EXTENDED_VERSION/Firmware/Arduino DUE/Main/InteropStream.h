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

    #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
      String getSKpart()
      {
        return PROTECT_KEY4;
      }
    #endif  
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
  bool canWorkWithPin;

  uint32_t _timer;

  
  public:
    BlinkModeInterop();

    void out(uint8_t level);
    void begin(uint8_t pin); // запоминаем настройки
    void blink(uint16_t interval=0); // мигаем диодом
    void update(); // обновляем состояние
};
//--------------------------------------------------------------------------------------------------------------------------------
#endif

