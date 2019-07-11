#ifndef _LOG_MODULE_H
#define _LOG_MODULE_H
#include "AbstractModule.h"
#include "Globals.h"
#include "RTCSupport.h"

#if TARGET_BOARD == STM32_BOARD
#include <SdFatSTM32.h>
#else
#include <SdFat.h>
#endif
//--------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  AbstractModule* RaisedModule; // модуль, который инициировал событие
  String Message; // действие, которое было произведено
  
} LogAction; // структура с описанием действий, которые произошли 
//--------------------------------------------------------------------------------------------------------------------------------
class LogModule : public AbstractModule // модуль логгирования данных с датчиков
{
  private:

  static String _COMMA;
  static String _NEWLINE;

  unsigned long lastUpdateCall;

  int8_t lastDOW;

  void writeToFile(SdFile& f, const String& data);

  SdFile logFile; // текущий файл для логгирования
  SdFile actionFile; // файл с записями о произошедших действиях
  String currentLogFileName; // текущее имя файла, с которым мы работаем сейчас
  unsigned long loggingInterval; // интервал между логгированиями

#ifdef LOG_ACTIONS_ENABLED
  int8_t lastActionsDOW;
  void EnsureActionsFileCreated(); // убеждаемся, что файл с записями текущих действий создан
  void CreateActionsFile(const RTCTime& tm); // создаёт новый файл лога с записью действий
#endif

  void CreateNewLogFile(const RTCTime& tm);
  void GatherLogInfo(const RTCTime& tm); 
#ifdef ADD_LOG_HEADER  
  void TryAddFileHeader();
#endif  

  String csv(const String& input);

  // HH:MM,MODULE_NAME,SENSOR_TYPE,SENSOR_IDX,SENSOR_DATA\r\n
  void WriteLogLine(const String& hhmm, const String& moduleName, const String& sensorType, const String& sensorIdx, const String& sensorData);
  
  public:
    LogModule() : AbstractModule("LOG") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

    void WriteAction(const LogAction& action); // записывает действие в файл событий

};
//--------------------------------------------------------------------------------------------------------------------------------
#endif
