#include "LogModule.h"
#include "ModuleController.h"
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef LOGGING_DEBUG_MODE
  #define LOG_DEBUG_WRITE(s) DEBUG_LOGLN((s))
#endif 
//--------------------------------------------------------------------------------------------------------------------------------
#define WRITE_TO_LOG(str) writeToFile(logFile,str)
#define WRITE_TO_ACTION_LOG(str) writeToFile(actionFile,str)
//--------------------------------------------------------------------------------------------------------------------------------
String LogModule::_COMMA;
String LogModule::_NEWLINE;
//--------------------------------------------------------------------------------------------------------------------------------
void LogModule::writeToFile(SdFile& f, const String& data)
{
  for(size_t i=0;i<data.length();i++)
  {
    f.write(data[i]);
    yield();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------
void LogModule::Setup()
{
    LogModule::_COMMA = COMMA_DELIMITER;
    LogModule::_NEWLINE = NEWLINE;

    currentLogFileName.reserve(20); // резервируем память, чтобы избежать фрагментации

   lastUpdateCall = 0;

   lastDOW = -1;
   
#ifdef LOG_ACTIONS_ENABLED   
   lastActionsDOW = -1;
#endif   

   loggingInterval = LOGGING_INTERVAL; // по умолчанию, берём из Globals.h. Позже - будет из настроек.
  // настройка модуля тут
 }
 //--------------------------------------------------------------------------------------------------------------------------------
#ifdef LOG_ACTIONS_ENABLED 
void LogModule::CreateActionsFile(const RTCTime& tm)
{  
  if(!MainController->HasSDCard())//hasSD)
    return;
  
  // формат YYYYMMDD.LOG
   String logFileName;
   logFileName += String(tm.year);

   if(tm.month < 10)
    logFileName += F("0");
   logFileName += String(tm.month);

   if(tm.dayOfMonth < 10)
    logFileName += F("0");
   logFileName += String(tm.dayOfMonth);

   logFileName += F(".LOG");

   String logDirectory = ACTIONS_DIRECTORY; // папка с логами действий

   if(!SDFat.exists(logDirectory.c_str())) // нет папки ACTIONS_DIRECTORY
   {
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("Creating the actions directory..."));
    #endif
      
      SDFat.mkdir(logDirectory.c_str()); // создаём папку
   }

 if(!SDFat.exists(logDirectory.c_str())) // проверяем её существование, на всякий
  {
    // не удалось создать папку actions
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("Unable to access to actions directory!"));
    #endif

    return;
  }

  logFileName = logDirectory + String(F("/")) + logFileName; // формируем полный путь

  if(actionFile.isOpen())
  {
    // уже есть открытый файл, проверяем, не пытаемся ли мы открыть файл с таким же именем
    String existingName = FileUtils::GetFileName(actionFile);
    if(logFileName.endsWith(existingName)) // такой же файл
      return; 
    else
      actionFile.close(); // закрываем старый
  } // if
  
  actionFile.open(logFileName.c_str(),FILE_WRITE); // открываем файл
   
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void LogModule::WriteAction(const LogAction& action)
{
#ifdef LOG_ACTIONS_ENABLED  
  EnsureActionsFileCreated(); // убеждаемся, что файл создан
  if(!actionFile.isOpen())
  {
    // что-то пошло не так
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("NO ACTIONS FILE AVAILABLE!"));
    #endif

    return;
  }

  #ifdef LOGGING_DEBUG_MODE
  LOG_DEBUG_WRITE( String(F("Write the \"")) + action.Message + String(F("\" action...")));
  #endif


#ifdef USE_DS3231_REALTIME_CLOCK

  RTCTime tm = /*rtc*/MainController->GetClock().getTime();
  
  String hhmm;
  if(tm.hour < 10)
     hhmm += F("0");
  hhmm += String(tm.hour);

  hhmm += F(":");
  
  if(tm.minute < 10)
    hhmm += F("0");
  hhmm += String(tm.minute);

  WRITE_TO_ACTION_LOG(hhmm); 
  WRITE_TO_ACTION_LOG(LogModule::_COMMA);
  WRITE_TO_ACTION_LOG(String(action.RaisedModule->GetID()));
  WRITE_TO_ACTION_LOG(LogModule::_COMMA);
  WRITE_TO_ACTION_LOG(csv(action.Message));
  WRITE_TO_ACTION_LOG(LogModule::_NEWLINE);

  actionFile.flush(); // сливаем данные на диск
#else
  UNUSED(action);  
#endif
  
  yield(); // даём поработать другим модулям
#else
UNUSED(action);
#endif
  
}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef LOG_ACTIONS_ENABLED
void LogModule::EnsureActionsFileCreated()
{
#ifdef USE_DS3231_REALTIME_CLOCK
  
  RTCTime tm = /*rtc*/MainController->GetClock().getTime();

  if(tm.dayOfWeek != lastActionsDOW)
  {
    // перешли на другой день недели, создаём новый файл
    lastActionsDOW = tm.dayOfWeek;
    
    if(actionFile.isOpen())
      actionFile.close();
      
    CreateActionsFile(tm); // создаём новый файл
  }
#endif 
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void LogModule::CreateNewLogFile(const RTCTime& tm)
{
  if(!MainController->HasSDCard())//hasSD)
    return;
  
    if(logFile.isOpen()) // есть открытый файл
      logFile.close(); // закрываем его

   // формируем имя нашего нового лог-файла:
   // формат YYYYMMDD.LOG

   currentLogFileName = String(tm.year);

   if(tm.month < 10)
    currentLogFileName += F("0");
    
   currentLogFileName += String(tm.month);
   
   if(tm.dayOfMonth < 10)
    currentLogFileName += F("0");
    
   currentLogFileName += String(tm.dayOfMonth);

   currentLogFileName += F(".LOG");

   String logDirectory = LOGS_DIRECTORY; // папка с логами
   if(!SDFat.exists(logDirectory.c_str())) // нет папки LOGS_DIRECTORY
   {
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("Creating the logs directory..."));
    #endif
      
      SDFat.mkdir(logDirectory.c_str()); // создаём папку
   }
   
  if(!SDFat.exists(logDirectory.c_str())) // проверяем её существование, на всякий
  {
    // не удалось создать папку logs
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("Unable to access to logs directory!"));
    #endif

    return;
  } 

   #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(String(F("Creating the ")) + currentLogFileName + String(F(" log file...")));
   #endif

   // теперь можем создать файл - даже если он существует, он откроется на запись
   currentLogFileName = logDirectory + String(F("/")) + currentLogFileName; // формируем полный путь

   logFile.open(currentLogFileName.c_str(),FILE_WRITE);

   if(logFile.isOpen())
   {
   #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(String(F("File ")) + currentLogFileName + String(F(" successfully created!")));
   #endif
    
   }
   else
   {
   #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(String(F("Unable to create the ")) + currentLogFileName + String(F(" log file!")));
   #endif
    return;
   }

   // файл создали, можем с ним работать.
#ifdef ADD_LOG_HEADER
   TryAddFileHeader(); // пытаемся добавить заголовок в файл
#endif   
      
}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef ADD_LOG_HEADER
void LogModule::TryAddFileHeader()
{
  uint32_t sz = logFile.size();
  if(!sz) // файл пуст
  {
   #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(String(F("Adding the header to the ")) + String(logFile.name()) + String(F(" file...")));
   #endif

    // тут можем добавлять свой заголовок в файл

    // сначала опрашиваем все модули в системе, записывая их имена в файл, и попутно сохраняя те типы датчиков, которые есть у модулей
    int statesFound = 0; // какие состояния нашли

    size_t cnt = MainController->GetModulesCount();
    bool anyModuleNameWritten = false;
   
    for(size_t i=0;i<cnt;i++)
    {
      AbstractModule* m = MainController->GetModule(i);
      if(m == this) // пропускаем себя
        continue;

      // смотрим, есть ли хоть одно интересующее нас состояние, попутно сохраняя найденные нами состояния в список найденных состояний
        bool anyInterestedStatesFound = false;
        if(m->State.HasState(StateTemperature))
        {
          statesFound |= StateTemperature;
          anyInterestedStatesFound = true;
        }
        if(m->State.HasState(StateLuminosity))
        {
          statesFound |= StateLuminosity;
          anyInterestedStatesFound = true;
        }
        if(m->State.HasState(StateHumidity))
        {
          statesFound |= StateHumidity;
          anyInterestedStatesFound = true;
        }
        if(m->State.HasState(StateWaterFlowIncremental))
        {
          statesFound |= StateWaterFlowIncremental;
          anyInterestedStatesFound = true;
        }
        if(m->State.HasState(StateSoilMoisture))
        {
          statesFound |= StateSoilMoisture;
          anyInterestedStatesFound = true;
        }
        if(m->State.HasState(StatePH))
        {
          statesFound |= StatePH;
          anyInterestedStatesFound = true;
        }
        
        if(anyInterestedStatesFound)
        {
          // есть интересующие нас состояния, можно писать в файл имя модуля и его индекс.
          if(anyModuleNameWritten) // уже было записано имя модуля, надо добавить запятую
          {
            WRITE_TO_LOG(LogModule::_COMMA);
          }
          
          String mName = m->GetID(); // получаем имя модуля
          mName += COMMAND_DELIMITER;
          mName += String(i);

          // строка вида MODULE_NAME=IDX сформирована, можно писать её в файл
          WRITE_TO_LOG(mName);
          anyModuleNameWritten = true;
        }
    } // for

    if(anyModuleNameWritten) // записали по крайней мере имя одного модуля, надо добавить перевод строки
          WRITE_TO_LOG(LogModule::_NEWLINE);

    // теперь можем писать в файл вторую строку, с привязками типов датчиков к индексам
    String secondLine;
 
    if(statesFound & StateTemperature) // есть температура
    {
      if(secondLine.length())
        secondLine += LogModule::_COMMA;

      secondLine += LOG_TEMP_TYPE;
      secondLine += COMMAND_DELIMITER;
      secondLine += String(StateTemperature);
    }
  
    if(statesFound & StateLuminosity) // есть освещенность
    {
      if(secondLine.length())
        secondLine += LogModule::_COMMA;
        
      secondLine += LOG_LUMINOSITY_TYPE;
      secondLine += COMMAND_DELIMITER;
      secondLine += String(StateLuminosity);
    }

    if(statesFound & StateHumidity) // есть влажность
    {
      if(secondLine.length())
        secondLine += LogModule::_COMMA;
        
      secondLine += LOG_HUMIDITY_TYPE;
      secondLine += COMMAND_DELIMITER;
      secondLine += String(StateHumidity);
    }

    if(statesFound & StateWaterFlowIncremental) // // есть датчик постоянного расхода воды
    {
      if(secondLine.length())
        secondLine += LogModule::_COMMA;
        
      secondLine += LOG_WATERFLOW_TYPE;
      secondLine += COMMAND_DELIMITER;
      secondLine += String(StateWaterFlowIncremental);
    }

    if(statesFound & StateSoilMoisture) // есть влажность почвы
    {
      if(secondLine.length())
        secondLine += LogModule::_COMMA;
        
      secondLine += LOG_SOIL_TYPE;
      secondLine += COMMAND_DELIMITER;
      secondLine += String(StateSoilMoisture);
    }    

    if(statesFound & StatePH) // есть pH
    {
      if(secondLine.length())
        secondLine += LogModule::_COMMA;
        
      secondLine += LOG_PH_TYPE;
      secondLine += COMMAND_DELIMITER;
      secondLine += String(StatePH);
    }    
    
   if(secondLine.length()) // можем записывать в файл вторую строку с привязкой датчиков к типам
   {
    secondLine += LogModule::_NEWLINE;
    WRITE_TO_LOG(secondLine);
   }

  // записали, выдыхаем, курим, пьём пиво :)
   #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("File header written successfully!"));
   #endif

   logFile.flush(); // сливаем данные на карту
   
   yield(); // т.к. запись на SD-карту у нас может занимать какое-то время - дёргаем кооперативный режим
    
  } // if(!sz) - файл пуст
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void LogModule::GatherLogInfo(const RTCTime& tm)
{
  // собираем информацию в лог
  
  if(!logFile.isOpen()) // что-то пошло не так
  {
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("Current log file not open!"));
    #endif
    return;
  }
  
  logFile.flush(); // сливаем информацию на карту
  
  yield(); // т.к. запись на SD-карту у нас может занимать какое-то время - дёргаем кооперативный режим
 
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("Gathering sensors data..."));
    #endif

  // строка с данными у нас имеет вид:
  // HH:MM,MODULE_NAME,SENSOR_TYPE,SENSOR_IDX,SENSOR_DATA
  // и соответствует формату CSV, т.е. если в данных есть "," и другие запрещенные символы, то данные обрамляются двойными кавычками
  String hhmm;
  if(tm.hour < 10)
     hhmm += F("0");
  hhmm += String(tm.hour);

  hhmm += F(":");
  
  if(tm.minute < 10)
    hhmm += F("0");
  hhmm += String(tm.minute);

// формируем типы данных, чтобы не дёргать их каждый раз в цикле
  String temperatureType = 
  #ifdef LOG_CHANGE_TYPE_TO_IDX
  String(StateTemperature);
  #else
  LOG_TEMP_TYPE;
  #endif

  String humidityType = 
  #ifdef LOG_CHANGE_TYPE_TO_IDX
  String(StateHumidity);
  #else
  LOG_HUMIDITY_TYPE;
  #endif

  String luminosityType = 
  #ifdef LOG_CHANGE_TYPE_TO_IDX
  String(StateLuminosity);
  #else
  LOG_LUMINOSITY_TYPE;
  #endif

  String waterflowType = 
  #ifdef LOG_CHANGE_TYPE_TO_IDX
  String(StateWaterFlowIncremental);
  #else
  LOG_WATERFLOW_TYPE;
  #endif

  String soilMoistureType = 
  #ifdef LOG_CHANGE_TYPE_TO_IDX
  String(StateSoilMoisture);
  #else
  LOG_SOIL_TYPE;
  #endif
 
  String phType = 
  #ifdef LOG_CHANGE_TYPE_TO_IDX
  String(StatePH);
  #else
  LOG_PH_TYPE;
  #endif

  // чтобы не дублировать код, который отличается только типом для датчика, который надо вывести в файл -
  // делаем два вектора - в одном из них будет нужный тип датчика, в другом - указатель на строку,
  // в котором содержится текстовое представление типа
  Vector<ModuleStates> statesTypes;
  Vector<String*> statesStrings;
  // теперь добавляем в эти вектора нужные состояния, которые мы хотим писать в лог.
  statesTypes.push_back(StateTemperature); statesStrings.push_back(&temperatureType);
  statesTypes.push_back(StateHumidity); statesStrings.push_back(&humidityType);
  statesTypes.push_back(StateLuminosity); statesStrings.push_back(&luminosityType);
  statesTypes.push_back(StateWaterFlowIncremental); statesStrings.push_back(&waterflowType);
  statesTypes.push_back(StateSoilMoisture); statesStrings.push_back(&soilMoistureType);
  statesTypes.push_back(StatePH); statesStrings.push_back(&phType);
 
  // он сказал - поехали
  size_t cnt = MainController->GetModulesCount();
  // он махнул рукой

    String moduleName;
    for(size_t i=0;i<cnt;i++)
    {
      AbstractModule* m = MainController->GetModule(i);
      if(m == this) // пропускаем себя
        continue;

        // смотрим, чего там есть у модуля
        moduleName = 
        #ifdef LOG_CNANGE_NAME_TO_IDX
        String(i);
        #else
        m->GetID();
        #endif

        // теперь последовательно проходим по вектору состояний, проверяя, есть ли такое
        String sensorIdx, stateType;
        for(size_t j=0;j<statesTypes.size();j++)
        {
            ModuleStates state = statesTypes[j]; // получили тип интересующего нас состояния            
            uint8_t stateCnt = m->State.GetStateCount(state);
            
            if(stateCnt > 0)
            {
              stateType = *(statesStrings[j]); // получили его строковое представление   
                         
              // есть такое состояние у модуля, записываем его, проходя по всем датчикам
              for(uint8_t stateIdx = 0; stateIdx < stateCnt;stateIdx++)
              {
                  OneState* os = m->State.GetStateByOrder(state,stateIdx);
                  if(os)
                  {
                      sensorIdx = String(os->GetIndex());
                      
                      #ifndef WRITE_ABSENT_SENSORS_DATA
                      if(os->HasData()) 
                      #endif
                      {
                          // пишем строку с данными               
                          WriteLogLine(hhmm,moduleName,stateType,sensorIdx,*os);
                      } // if                      
                  } // if (os)
              } // for
              
            } // stateCnt > 0
            
        } // for
                
    } // for

  
    // записали, выдохнули, расслабились.
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("Sensors data gathered."));
    #endif
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void LogModule::WriteLogLine(const String& hhmm, const String& moduleName, const String& sensorType, const String& sensorIdx, const String& sensorData)
{
  // пишем строку с данными в лог
  // HH:MM,MODULE_NAME,SENSOR_TYPE,SENSOR_IDX,SENSOR_DATA\r\n
  WRITE_TO_LOG(hhmm);             WRITE_TO_LOG(LogModule::_COMMA);
  WRITE_TO_LOG(moduleName);       WRITE_TO_LOG(LogModule::_COMMA);
  WRITE_TO_LOG(sensorType);       WRITE_TO_LOG(LogModule::_COMMA);
  WRITE_TO_LOG(sensorIdx);        WRITE_TO_LOG(LogModule::_COMMA);
  WRITE_TO_LOG(csv(sensorData));  WRITE_TO_LOG(LogModule::_NEWLINE);

  logFile.flush(); // сливаем данные на карту

  yield(); // т.к. запись на SD-карту у нас может занимать какое-то время - дёргаем кооперативный режим

}
//--------------------------------------------------------------------------------------------------------------------------------
String LogModule::csv(const String& src)
{
  String fnd = F("\"");
  String rpl = fnd + fnd;
  String input = src;
  input.replace(fnd,rpl); // заменяем кавычки на двойные
  
 if(input.indexOf(LogModule::_COMMA) != -1 ||
    input.indexOf(F("\"")) != -1 ||
    input.indexOf(F(";")) != -1 ||
    input.indexOf(F(",")) != -1 || // прописываем запятую принудительно, т.к. пользователь может переопределить COMMA_DELIMITER
    input.indexOf(LogModule::_NEWLINE) != -1
 )
 { // нашли запрещённые символы - надо обрамить в двойные кавычки строку
  
  String s; s.reserve(input.length() + 2);
  s += fnd;
  s += input;
  s += fnd;
  
  return s;
 }

  return input;
}
//--------------------------------------------------------------------------------------------------------------------------------
void LogModule::Update(uint16_t dt)
{ 
  lastUpdateCall += dt;
  if(lastUpdateCall < loggingInterval) // не надо обновлять ничего - не пришло время
    return;
  else
    lastUpdateCall = 0;

  if(!MainController->HasSDCard())//hasSD) // нет карты или карту не удалось инициализировать
  {
    #ifdef LOGGING_DEBUG_MODE
    LOG_DEBUG_WRITE(F("NO SD CARD PRESENT!"));
    #endif
    return;
  }
#ifdef USE_DS3231_REALTIME_CLOCK
  RTCTime tm = /*rtc*/MainController->GetClock().getTime();
  if(lastDOW != tm.dayOfWeek) // наступил следующий день недели, надо создать новый лог-файл
  {
   lastDOW = tm.dayOfWeek;
   CreateNewLogFile(tm); // создаём новый файл
#ifdef LOG_ACTIONS_ENABLED
   EnsureActionsFileCreated(); // создаём новый файл действий, если он ещё не был создан
#endif
  }

  GatherLogInfo(tm); // собираем информацию в лог
#endif    
  // обновление модуля тут

}
//--------------------------------------------------------------------------------------------------------------------------------
bool LogModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);

  PublishSingleton = UNKNOWN_COMMAND;
  size_t argsCnt = command.GetArgsCount();

if(MainController->HasSDCard())//hasSD)
{
  if(command.GetType() == ctSET) 
  {
    PublishSingleton = NOT_SUPPORTED;
  }
  else
  {
    if(argsCnt > 0)
    {
      String cmd = command.GetArg(0);
      if(cmd == FILE_COMMAND)
      {
        // надо отдать файл
        if(argsCnt > 1)
        {
          // получаем полное имя файла
          String fileNameRequested = command.GetArg(1);
          String fullFilePath = LOGS_DIRECTORY;
          fullFilePath += F("/");
          fullFilePath += fileNameRequested;

          if(SDFat.exists(fullFilePath.c_str()))
          {
            // такой файл существует, можно отдавать
            if(logFile.isOpen())
              logFile.close(); // сперва закрываем текущий лог-файл

            // теперь можно открывать файл на чтение
            SdFile fRead;
            if(fRead.open(fullFilePath.c_str(),FILE_READ))
            {
              // файл открыли, можно читать
              // сперва отправим в потом строчку OK=FOLLOW
              Stream* writeStream = command.GetIncomingStream();
              writeStream->print(OK_ANSWER);
              writeStream->print(COMMAND_DELIMITER);
              writeStream->println(FOLLOW);

              //теперь читаем из файла блоками, делая паузы для вызова yield через несколько блоков
              const int DELAY_AFTER = 2;
              int delayCntr = 0;

              uint16_t readed;

               while(fRead.available())
               {
                readed = fRead.read(SD_BUFFER,SD_BUFFER_LENGTH);
                writeStream->write(SD_BUFFER,readed);
                delayCntr++;
                if(delayCntr > DELAY_AFTER)
                {
                  delayCntr = 0;
                  yield(); // даём поработать другим модулям
                }
               } // while
              
              
              fRead.close(); // закрыли файл
              PublishSingleton.Flags.Status = true;
              PublishSingleton = END_OF_FILE; // выдаём OK=END_OF_FILE
            } // if(fRead)

            #ifdef USE_DS3231_REALTIME_CLOCK
                RTCTime tm = MainController->GetClock().getTime();
                CreateNewLogFile(tm); // создаём новый файл
            #endif
            
          } // SDFat.exists
          
        } // if(argsCnt > 1)
        else
        {
          PublishSingleton = PARAMS_MISSED;
        }
        
      } // FILE_COMMAND
      else
      if(cmd == ACTIONS_COMAND)
      {
        // надо отдать файл действий
        if(argsCnt > 1)
        {
          // получаем полное имя файла
          String fileNameRequested = command.GetArg(1);
          String fullFilePath = ACTIONS_DIRECTORY;
          fullFilePath += F("/");
          fullFilePath += fileNameRequested;

          if(SDFat.exists(fullFilePath.c_str()))
          {
            // такой файл существует, можно отдавать
            if(actionFile.isOpen())
              actionFile.close(); // сперва закрываем текущий файл действий

            // теперь можно открывать файл на чтение
            SdFile fRead;
            if(fRead.open(fullFilePath.c_str(),FILE_READ))
            {
              // файл открыли, можно читать
              // сперва отправим в потом строчку OK=FOLLOW
              Stream* writeStream = command.GetIncomingStream();
              writeStream->print(OK_ANSWER);
              writeStream->print(COMMAND_DELIMITER);
              writeStream->println(FOLLOW);

              //теперь читаем из файла блоками, делая паузы для вызова yield через несколько блоков
              const int DELAY_AFTER = 2;
              int delayCntr = 0;

              uint16_t readed;

               while(fRead.available())
               {
                readed = fRead.read(SD_BUFFER,SD_BUFFER_LENGTH);
                writeStream->write(SD_BUFFER,readed);
                delayCntr++;
                if(delayCntr > DELAY_AFTER)
                {
                  delayCntr = 0;
                  yield(); // даём поработать другим модулям
                }
               } // while
              
              
              fRead.close(); // закрыли файл
              PublishSingleton.Flags.Status = true;
              PublishSingleton = END_OF_FILE; // выдаём OK=END_OF_FILE
            } // if(fRead)

            #if defined(USE_DS3231_REALTIME_CLOCK) && defined(LOG_ACTIONS_ENABLED)
                RTCTime tm = MainController->GetClock().getTime();
                CreateActionsFile(tm); // создаём новый файл действий
            #endif
            
          } // SDFat.exists
          
        } // if(argsCnt > 1)
        else
        {
          PublishSingleton = PARAMS_MISSED;
        }
        
      } // ACTIONS_COMAND
      else
      {
        PublishSingleton = UNKNOWN_COMMAND;
      }
       
    } // argsCnt > 0
  } // ctGET
  
} // hasSD
  
  // отвечаем на команду
  MainController->Publish(this,command);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------
