#pragma once

#include "AbstractModule.h"
#include "Events.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class ScheduleModule : public AbstractModule // модуль расписания
{
  private:

    void processActiveSchedule();
  
  public:
    ScheduleModule() : AbstractModule("SCHEDULE") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

};
//--------------------------------------------------------------------------------------------------------------------------------------

