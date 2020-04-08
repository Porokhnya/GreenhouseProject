#ifndef _MODULE_CONTROLLER_H
#define _MODULE_CONTROLLER_H

#include <Arduino.h>
#include "Globals.h"
#include "AbstractModule.h"
#include "CommandParser.h"
#include "TinyVector.h"
#include "Settings.h"
#include "AlarmDispatcher.h"


#ifdef USE_DS3231_REALTIME_CLOCK
#include "RTCSupport.h"
#endif

#ifdef USE_LOG_MODULE
#include "LogModule.h"
#endif

#include "HTTPInterfaces.h" // подключаем интерфейсы для работы с HTTP-запросами


#if TARGET_BOARD == STM32_BOARD
#include <SdFatSTM32.h>
#else
#include <SdFat.h>
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
class AbstractModule; // forward declaration
class AlertRule;
typedef Vector<AbstractModule*> ModulesVec;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef void (*CallbackUpdateFunc)(AbstractModule* mod);
//--------------------------------------------------------------------------------------------------------------------------------------
class FileUtils
{
  public:
  
     static void readLine(SdFile& f, String& result);

     static String GetFileName(SdFile& f);
     static int CountFiles(const String& dirName, bool recursive=true);
     static void RemoveFiles(const String& dirName, bool recursive=true);

};
//--------------------------------------------------------------------------------------------------------------------------------------
class ModuleController
{
 private:
  ModulesVec modules; // список зарегистрированных модулей
  
  CommandParser* cParser; // парсер текстовых команд

  GlobalSettings settings; // глобальные настройки

  ReservationResolver* reservationResolver; // держатель списков резервирования

  HTTPQueryProvider* httpQueryProviders[2];

#ifdef USE_DS3231_REALTIME_CLOCK
  RealtimeClock _rtc; // часы реального времени
#endif

#ifdef USE_LOG_MODULE
  LogModule* logWriter;
#endif

#ifdef SD_USED
  bool sdCardInitFlag;
#endif

  void PublishToCommandStream(AbstractModule* module,const Command& sourceCommand); // публикация в поток команды
  void streamWrite(Stream* s, const String& str);

#ifdef USE_ALARM_DISPATCHER
  AlarmDispatcher alarmDispatcher;
#endif
  
public:
  ModuleController();

  void Setup(); // настраивает контроллер на работу (инициализация нужных железок и т.п.)
  void begin(); // начинаем работу

  // устанавливает обработчика списков резервирования
  void SetReservationResolver(ReservationResolver* rr) { reservationResolver = rr; }
  // возвращает состояние с зарезервированного списка для датчика модуля, с которого нет показаний
  OneState* GetReservedState(AbstractModule* sourceModule, ModuleStates sensorType, uint8_t sensorIndex);

  bool SDSupported()
  {
    #ifdef SD_USED
      return true;
    #else
      return false;
    #endif
  }

  bool HasSDCard() 
  {
#ifdef SD_USED
    return sdCardInitFlag;
#else
    return false;
#endif
  }
  #ifdef USE_DS3231_REALTIME_CLOCK
  // модуль реального времени
  RealtimeClock& GetClock();
  #endif

  #ifdef USE_LOG_MODULE
  void SetLogWriter(LogModule* lw) {logWriter = lw;}
  #endif

  void Log(AbstractModule* mod, const String& message); // добавляет строчку в лог действий

  // возвращает текущие настройки контроллера
  GlobalSettings* GetSettings() {return &settings;}
 
  size_t GetModulesCount() {return modules.size(); }
  AbstractModule* GetModule(size_t idx) {return modules[idx]; }
  AbstractModule* GetModuleByID(const String& id);

  void RegisterModule(AbstractModule* mod);
  void ProcessModuleCommand(const Command& c, AbstractModule* thisModule=NULL);
  
  void UpdateModules(CallbackUpdateFunc func);
  
  void Publish(AbstractModule* module,const Command& sourceCommand); // каждый модуль по необходимости дергает этот метод для публикации событий/ответов на запрос

  void SetCommandParser(CommandParser* c) {cParser = c;};
  CommandParser* GetCommandParser() {return cParser;}

  HTTPQueryProvider* GetHTTPProvider(byte idx) {return httpQueryProviders[idx]; }
  void SetHTTPProvider(byte idx, HTTPQueryProvider* prov) {httpQueryProviders[idx] = prov; }

  void Alarm(AlertRule* rule); // обработчик тревог
  void RemoveAlarm(AlertRule* rule); // обработчик тревог
  #ifdef USE_ALARM_DISPATCHER
    AlarmDispatcher* GetAlarmDispatcher(){ return &alarmDispatcher;}
  #endif

      #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
      String getSKpart()
      {
        return PROTECT_KEY3;
      }

      Vector<uint32_t> keyParts;
      void saveKeyParts(uint32_t part1, uint32_t part2, uint32_t part3, uint32_t part4);
    #endif

   bool checkReg();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern PublishStruct PublishSingleton; // сюда публикуем все ответы от всех модудей
extern ModuleController* MainController; // главный контроллер
extern SdFat SDFat;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
