#pragma once

#include <Arduino.h>
#include "TinyVector.h"
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
enum class Event
{
  NOP, // "никакое" событие
  
  SettingsChanged, // изменены настройки контроллера  
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
struct IEventSubscriber
{
  virtual void onEvent(Event event, void* param) = 0;
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<IEventSubscriber*> IEventSubscriberList;
//--------------------------------------------------------------------------------------------------------------------------------------
class EventsClass
{
  private:

    IEventSubscriberList list;
    String messageString;
  
public:
  EventsClass();

  void raise(Event event, void* param=NULL);
  void subscribe(IEventSubscriber* s);
  void unsubscribe(IEventSubscriber* s);

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern EventsClass Events;
