#ifndef _ALARM_DISPATCHER_H
#define _ALARM_DISPATCHER_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <WString.h>
#include "TinyVector.h"
#include "Globals.h"
#include "AlertModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
// классы и интерфейсы для управления тревогами
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<AlertRule*> RaisedAlarmsList;
//--------------------------------------------------------------------------------------------------------------------------------------
class AlarmDispatcher
{
  private:

#ifdef USE_SMS_MODULE
    RaisedAlarmsList queueSMSAlarms;
    RaisedAlarmsList processedSMSAlarms;   
#endif

  public:
    AlarmDispatcher();

    void RemoveAlarm(AlertRule* rule);
    void Alarm(AlertRule* rule);
    void ClearProcessedAlarms();

    #ifdef USE_SMS_MODULE
      // функции, специфичные для GSM-модуля
      bool HasSMSAlarm() {return queueSMSAlarms.size();}
      String GetSMSAlarmData();
      void MarkSMSAlarmDone();
    #endif
};
//--------------------------------------------------------------------------------------------------------------------------------------

#endif

