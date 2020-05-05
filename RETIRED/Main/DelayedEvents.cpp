#include "DelayedEvents.h"
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDelayedEventClass CoreDelayedEvent;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDelayedEventClass::CoreDelayedEventClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDelayedEventClass::raise(uint32_t raiseDelay,CoreDelayedEventHandler handler, void* param)
{
  CoreDelayedEventData rec;
  rec.timer = millis();
  rec.duration = raiseDelay;
  rec.handler = handler;
  rec.param = param;
  
  signals.push_back(rec);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDelayedEventClass::removeByParam(CoreDelayedEventHandler handler, void* param)
{
  CoreDelayedEventsList cpy;
  for(size_t i=0;i<signals.size();i++)
  {
    if(!(signals[i].handler == handler && signals[i].param == param))
    {
      cpy.push_back(signals[i]);
    }
  }

  signals = cpy;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDelayedEventClass::update()
{
    for(size_t i=0;i<signals.size();)
    {
        if(millis() - signals[i].timer > signals[i].duration)
        {
          // сигнал сработал
          CoreDelayedEventHandler handler = signals[i].handler;
          void* param = signals[i].param;

          // сначала мы убираем его из очереди
          for(size_t j=i+1; j< signals.size(); j++)
          {
            signals[j-1] = signals[j];
          }

          signals.pop();

          // и только потом вызываем обработчик сигнала
          if(handler)
            handler(param);
            
        } // if
        else
        {
          i++;
        }
    } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDelayedEventClass::CoreDelayedEventPinChange(void* param)
{
    
  CoreDelayedEventPinChangeArg* arg = (CoreDelayedEventPinChangeArg*) param;
  pinMode(arg->pin,OUTPUT);
  digitalWrite(arg->pin,arg->level);

  delete arg;
}
//--------------------------------------------------------------------------------------------------------------------------------------

