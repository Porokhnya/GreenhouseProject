#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef void (*CoreDelayedEventHandler)(void* param);
//--------------------------------------------------------------------------------------------------------------------------------------
struct CoreDelayedEventPinChangeArg
{
  uint8_t pin;
  uint8_t level;
  
  CoreDelayedEventPinChangeArg(uint8_t p, uint8_t l)
  {
    pin = p;
    level = l;
  }
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint32_t timer;
  uint32_t duration;
  void* param;
  CoreDelayedEventHandler handler;
  
} CoreDelayedEventData;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreDelayedEventData> CoreDelayedEventsList;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreDelayedEventClass
{
  public:
    CoreDelayedEventClass();

    void update();
    void raise(uint32_t raiseDelay,CoreDelayedEventHandler handler, void* param);
    void removeByParam(CoreDelayedEventHandler handler, void* param);

    static void CoreDelayedEventPinChange(void* param);

  private:

    CoreDelayedEventsList signals;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreDelayedEventClass CoreDelayedEvent;
//--------------------------------------------------------------------------------------------------------------------------------------

