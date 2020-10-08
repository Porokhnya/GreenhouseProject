#pragma once

#include "AbstractModule.h"
#include "Events.h"
#include "RTCSupport.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class ScheduleModule : public AbstractModule // модуль расписания
{
  private:

    void processActiveSchedule();
    uint32_t getScheduleCount(const Command& command);
    String getScheduleDate(const Command& command);
    String getScheduleContent(const Command& command);
    void deleteScheduleByDate(const Command& command);
    bool addSchedule(const Command& command);

    bool isNowSchedule(const String& mmddhhmm);
    void processSchedule(const String& fileName);

    String getNowStr(RTCTime& tm);
    void processScheduleFolder(const String& dirName,const String& wantedScheduleFileName);

    void processActivityCommand(const Command& command);

    //RTCTime lastTime;
    uint32_t timer;
  
  public:
    ScheduleModule() : AbstractModule("SCHEDULE") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

};
//--------------------------------------------------------------------------------------------------------------------------------------

