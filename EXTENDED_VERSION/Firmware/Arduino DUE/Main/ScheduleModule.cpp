#include "ScheduleModule.h"
#include "ModuleController.h"
#include "Settings.h"
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::processActiveSchedule()
{
    // тут обработка активного расписания

    //TODO: ПОКА ЗАТЫЧКА ДЛЯ ПРОВЕРКИ УСПЕШНОСТИ КОМПИЛЯЦИИ, УБРАТЬ !!!
    Events.raise(Event::SettingsChanged); // типа, настройки изменились
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::Setup()
{
  // настройка модуля тут
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::Update()
{ 
  static bool bFirst = true;

  GlobalSettings* sett = MainController->GetSettings();

  if(!sett->isScheduleActive()) // модуль расписания неактивен, не надо работать
  {
    bFirst = false;
    return;
  }

  // при первом обновлении модуля - сразу обрабатываем активное расписание
  if(bFirst)
  {
    bFirst = false;

    processActiveSchedule();
    
    return;

  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  ScheduleModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  PublishSingleton = UNKNOWN_COMMAND;

  uint8_t argsCnt = command.GetArgsCount();

  if(command.GetType() == ctSET) // SET
  {
    if(argsCnt > 0)
    {
        String s = command.GetArg(0);
        if(s == F("ACTIVE")) // CTSET=SCHEDULE|ACTIVE|flag
        {
            if(argsCnt > 1)
            {
              uint8_t val = atoi(command.GetArg(1));
              GlobalSettings* sett = MainController->GetSettings();
              sett->setScheduleActive(val == 1);

              PublishSingleton.Flags.Status = true;
              PublishSingleton = command.GetArg(0);
              PublishSingleton << PARAM_DELIMITER << val;
            }
        } // ACTIVE
    } //  if(argsCnt > 0)
    
  } // SET
  else // GET
  {

       if(argsCnt > 0)
       {
          String s = command.GetArg(0);
          if(s == F("ACTIVE")) // CTGET=SCHEDULE|ACTIVE
          {
              GlobalSettings* sett = MainController->GetSettings();
              PublishSingleton.Flags.Status = true;
              PublishSingleton = s;
              PublishSingleton << PARAM_DELIMITER << (sett->isScheduleActive() ? 1 : 0);
              
          } // ACTIVE
        
       } //  if(argsCnt > 0)
    
  } // GET

    // отвечаем на команду
    MainController->Publish(this,command);
    
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------


