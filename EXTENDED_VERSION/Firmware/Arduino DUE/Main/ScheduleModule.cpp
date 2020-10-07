#include "ScheduleModule.h"
#include "ModuleController.h"
#include "Settings.h"
#include "InteropStream.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#define SCHEDULE_FILE_EXT F(".TXT")
#define SCHEDULE_FOLDER F("SCHEDULE")
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::processScheduleFolder(const String& dirName,const String& wantedScheduleFileName)
{
   if(SDFat.exists(dirName.c_str()))
   {
      SdFile root;
      if(root.open(dirName.c_str(),O_READ))
      {
        SdFile entry;
        while(entry.openNext(&root,O_READ))
        {
          yield();
          if(entry.isDir())
          {
              String subPath = dirName + "/";
              subPath += FileUtils::GetFileName(entry);
              processScheduleFolder(subPath,wantedScheduleFileName);      
          }
          else
          {      
            String fName = FileUtils::GetFileName(entry);
            if(fName == wantedScheduleFileName)
            {
              String subPath = dirName + "/";
              subPath += FileUtils::GetFileName(entry);
              
              entry.close();
              processSchedule(subPath);
              break;
            }
          }
          entry.close();
        } // while        

        root.close();
      }
   }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::processActiveSchedule()
{
  if(!MainController->HasSDCard())
  {
    return; // нет SD-карточки, нечего выполнять
  }
#ifdef USE_DS3231_REALTIME_CLOCK  

    // тут обработка активного расписания
    String dirName = SCHEDULE_FOLDER;
    
   RealtimeClock rtc = MainController->GetClock();
   RTCTime tm = rtc.getTime();   

   GlobalSettings* sett = MainController->GetSettings();
   // проверяем, не выполняли ли мы на эту дату расписание?
   uint8_t month, dayOfMonth;
   uint16_t year;
   sett->getLastScheduleRunDate(dayOfMonth,month,year);

   if(!(dayOfMonth == tm.dayOfMonth && month == tm.month && year == tm.year)) // на сегодняшнюю дату расписания ещё не выполняли
   {
   
     String nowStr = getNowStr(tm);
     nowStr += SCHEDULE_FILE_EXT;
  
     processScheduleFolder(dirName,nowStr);
  
     // выполнили, и запоминаем, за какую дату мы выполнили задание последний раз, чтобы не выполнять его ещё раз
     sett->setLastScheduleRunDate(tm.dayOfMonth,tm.month,tm.year);
   }

    
#endif // USE_DS3231_REALTIME_CLOCK

}
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::Setup()
{
  timer = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::Update()
{ 
#ifdef USE_DS3231_REALTIME_CLOCK
  
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

  //Тут раз в минуту проверяем на смену даты, и если она сменилась - выполняем текущее задание
  if(millis() - timer >= 60000ul)
  {
    RealtimeClock rtc = MainController->GetClock();
    RTCTime tm = rtc.getTime();   

    if(tm.dayOfMonth != lastTime.dayOfMonth || tm.month != lastTime.month || tm.year != lastTime.year)
    {
      lastTime = tm;
      processActiveSchedule();
    }
   
    timer = millis();
  }
  
#endif // #ifdef USE_DS3231_REALTIME_CLOCK
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t ScheduleModule::getScheduleCount(const Command& command)
{
  if(!MainController->HasSDCard())
  {
    return 0;
  }

  String fileName = SCHEDULE_FOLDER;
  fileName += "/";
  fileName += command.GetArg(1);

  return FileUtils::CountFiles(fileName,false);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::deleteScheduleByDate(const Command& command)
{
  if(!MainController->HasSDCard())
  {
    return;
  }

   String fileName = SCHEDULE_FOLDER;
   fileName += "/";
   fileName += command.GetArg(1);
   fileName += "/";
   fileName += command.GetArg(2);
   fileName += SCHEDULE_FILE_EXT;

   if(!SDFat.exists(fileName.c_str()))
   {
    return;
   }

    SDFat.remove(fileName.c_str());
}
//--------------------------------------------------------------------------------------------------------------------------------------
String ScheduleModule::getNowStr(RTCTime& tm)
{
   String nowStr;
   if(tm.month < 10)
   {
    nowStr += '0';
   }
   nowStr += tm.month;

   if(tm.dayOfMonth < 10)
   {
    nowStr += '0';
   }
   nowStr += tm.dayOfMonth;

   return nowStr;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool ScheduleModule::isNowSchedule(const String& mmdd)
{
  #ifdef USE_DS3231_REALTIME_CLOCK

   RealtimeClock rtc = MainController->GetClock();
   RTCTime tm = rtc.getTime();
   
   String nowStr = getNowStr(tm);
   
   return (mmdd == nowStr);
  
  #else
    return false;
  #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ScheduleModule::processSchedule(const String& fileName)
{
  if(!MainController->HasSDCard())
  {
    return;
  }

  SdFile file;
  if(!file.open(fileName.c_str(),O_READ))
  {
    return;
  }    

  String command;
  while(file.available())
  {
    command += (char) file.read();
  }    
  file.close();

  if(command.length())
  {
    ModuleInterop.QueryCommand(ctSET,command,false);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------    
bool ScheduleModule::addSchedule(const Command& command)
{
  if(!MainController->HasSDCard())
  {
    return false;
  }

  String fileName = SCHEDULE_FOLDER;
  SDFat.mkdir(fileName.c_str());
  
  fileName += "/";  
  fileName += command.GetArg(1);
  SDFat.mkdir(fileName.c_str());

  fileName += "/";  
  fileName += command.GetArg(2);
  fileName += SCHEDULE_FILE_EXT;

  SdFile file;
  if(!file.open(fileName.c_str(),FILE_WRITE | O_TRUNC))
  {
    return false;
  }

  size_t to = command.GetArgsCount();
  String delim = PARAM_DELIMITER;
  for(size_t i=3;i<to;i++)
  {
    String param = command.GetArg(i);
    file.write((const uint8_t*) param.c_str(),param.length());
    if(i+1 < to)
    {
      file.write((const uint8_t*) delim.c_str(),delim.length());
    }
  } // for

  file.flush();
  file.close();

  if(isNowSchedule(command.GetArg(2))) // если это новое расписание на сегодня, то выполняем его
  {
    processSchedule(fileName);
  }

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
String ScheduleModule::getScheduleContent(const Command& command)
{
  String result;

  if(!MainController->HasSDCard())
  {
    return result;
  } 

   String fileName = SCHEDULE_FOLDER;
   fileName += "/";
   fileName += command.GetArg(1);
   fileName += "/";
   fileName += command.GetArg(2);
   fileName += SCHEDULE_FILE_EXT;

   if(!SDFat.exists(fileName.c_str()))
   {
    return result;
   }

  SdFile file;
  if(!file.open(fileName.c_str(),O_READ))
  {
    return result;  
  }

  while(file.available())
  {
    result += (char) file.read();
  }

  file.close();
  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
String ScheduleModule::getScheduleDate(const Command& command)
{
  String result;

  if(!MainController->HasSDCard())
  {
    return result;
  } 

   String dirName = SCHEDULE_FOLDER;
   dirName += "/";
   dirName += command.GetArg(1);
   
   uint32_t skipCounter = 0;
   uint32_t toSkip = atol(command.GetArg(2));

  SdFile root;
  if(!root.open(dirName.c_str(),FILE_READ))
  {
    return result;
  }

  root.rewind();

  SdFile entry;
  while(entry.openNext(&root,FILE_READ))
  {
    
    if(entry.isDir())
    {
    }
    else
    {
      skipCounter++;
      if(skipCounter > toSkip)
      { 
        String fileName = FileUtils::GetFileName(entry);
        int idx = fileName.indexOf('.');
        if(idx != -1)
        {
          result = fileName.substring(0,idx);
          entry.close();
          break;
        }
      }


    }
    entry.close();
  } // while
  
  root.close();
  return result;
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
        else
        if(s == F("DELETE")) // удалить расписание в папке за указанную дату, CTSET=SCHEDULE|DELETE|folder|mmdd
        {
            if(argsCnt > 2)
            {
              deleteScheduleByDate(command);
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = s;
              PublishSingleton << PARAM_DELIMITER << (command.GetArg(1))
              << PARAM_DELIMITER << (command.GetArg(2));
            }          
        } // DELETE
        else
        if(s == F("ADD")) // добавить расписание в папку на текущую дату, CTSET=SCHEDULE|ADD|folder|mmdd|schedule data here
        {
          if(argsCnt > 3)
          {
              if(addSchedule(command))
              {
                PublishSingleton.Flags.Status = true;
                PublishSingleton = s;
                PublishSingleton << PARAM_DELIMITER << (command.GetArg(1))
                << PARAM_DELIMITER << (command.GetArg(2))
                << PARAM_DELIMITER << REG_SUCC;            
              }
          }
        } // ADD
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
          else
          if(s == F("COUNT")) // количество заданий в определённой папке, CTGET=SCHEDULE|COUNT|folder
          {
            if(argsCnt > 1)
            {
              PublishSingleton.Flags.Status = true;
              PublishSingleton = s;
              PublishSingleton << PARAM_DELIMITER << (command.GetArg(1))
              << PARAM_DELIMITER << (getScheduleCount(command));
            }
            
          } // COUNT
          else
          if(s == F("DATE")) // вернуть дату задания в папке по порядку, CTGET=SCHEDULE|DATE|folder|num
          {
            if(argsCnt > 2)
            {
              PublishSingleton.Flags.Status = true;
              PublishSingleton = s;
              PublishSingleton << PARAM_DELIMITER << (command.GetArg(1))
              << PARAM_DELIMITER << (command.GetArg(2))
              << PARAM_DELIMITER << (getScheduleDate(command));
            }            
          } // DATE
          else
          if(s == F("READ")) // прочитать данные задания по папке и дате, CTGET=SCHEDULE|READ|folder|mmdd
          {
            if(argsCnt > 2)
            {
              PublishSingleton.Flags.Status = true;
              PublishSingleton = s;
              PublishSingleton << PARAM_DELIMITER << (command.GetArg(1))
              << PARAM_DELIMITER << (command.GetArg(2))
              << PARAM_DELIMITER << (getScheduleContent(command));
            }                        
          } // READ
        
       } //  if(argsCnt > 0)
    
  } // GET

    // отвечаем на команду
    MainController->Publish(this,command);
    
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------


