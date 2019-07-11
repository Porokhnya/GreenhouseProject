#include "WiFiModule.h"
#include "ModuleController.h"
#include "InteropStream.h"
#include "Memory.h"
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE
//--------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::EnsureIoTProcessed(bool success)
{
   if(iotDone) 
   {
      // да, нас вызывали для отсыла данных в IoT, но что-то пошло не так
      iotDone({success,iotService});
      iotDone = NULL;
      iotWriter = NULL;
   }    
}
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::sendDataToThingSpeak()
{
  thingSpeakDataWritten = false;
  
  if(!thingSpeakClient.connected() || !ESP.ready())
  {
    EnsureIoTProcessed(false);
    return;
  }

  // тут посылаем данные в клиент для ThingSpeak
  delete streamBuffer;
  streamBuffer = new String();
  // резервируем нужное кол-во памяти
  streamBuffer->reserve(thingSpeakDataLength + 150);

  IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();
  
  *streamBuffer = F("GET /update?headers=false&api_key=");
  *streamBuffer += iotSettings.ThingSpeakChannelID;
  *streamBuffer += F("&");

  // просим, чтобы записали данные в нас
  iotWriter(this);

  *streamBuffer += F(" HTTP/1.1\r\nAccept: */*\r\nUser-Agent: ");
  *streamBuffer += IOT_USER_AGENT;
  *streamBuffer += F("\r\nHost: ");
  *streamBuffer += THINGSPEAK_HOST;
  *streamBuffer += F("\r\n\r\n");

  #ifdef WIFI_DEBUG
    DEBUG_LOGLN(*streamBuffer);
  #endif

  // теперь пишем в клиент
  thingSpeakClient.write((uint8_t*) streamBuffer->c_str(),streamBuffer->length());

  // тут сразу можно удалять буфер, т.к. он уже скопирован в клиент
  delete streamBuffer;
  streamBuffer = new String();
}
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::SendData(IoTService service,uint16_t dataLength, IOT_OnWriteToStream writer, IOT_OnSendDataDone onDone)
{
  thingSpeakDataWritten = false;
  
  if(!ESP.ready())
  {
    onDone({false,service});
    return;
  }
  
    thingSpeakDataLength = dataLength;
  
    // тут смотрим, можем ли мы обработать запрос на отсыл данных в IoT
    IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();

    if(iotSettings.Flags.ThingSpeakEnabled && strlen(iotSettings.ThingSpeakChannelID)) // включен один сервис хотя бы
    {

     // сохраняем указатели на функции обратного вызова
      iotWriter = writer;
      iotDone = onDone;
      iotService = service;

      // Тут формируем данные для запроса
      switch(service)
      {
         case iotThingSpeak:
         {
          // попросили отослать данные через ThingSpeak          
          // пробуем законнектиться
          String tsIP = THINGSPEAK_IP;
          thingSpeakClient.connect(tsIP.c_str(),80);
         }
         break;
        
      } // switch
    } // enabled
    else
    {
      // ни одного сервиса не включено
      // тут ничего не можем отсылать, сразу дёргаем onDone, говоря, что у нас не получилось отослать
      onDone({false,service});

    }
}
//--------------------------------------------------------------------------------------------------------------------------------
#endif // defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
//--------------------------------------------------------------------------------------------------------------------------------
bool WiFiModule::CanMakeQuery() // тестирует, может ли модуль сейчас сделать запрос
{ 
 return ESP.ready();
}
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::MakeQuery(HTTPRequestHandler* handler) // начинаем запрос по HTTP
{
    // сперва завершаем обработку предыдущего вызова, если он вдруг нечаянно был
    EnsureHTTPProcessed(ERROR_HTTP_REQUEST_CANCELLED);

    // сохраняем обработчик запроса у себя
    httpHandler = handler;

    // спрашиваем о хосте у вызывающей стороны
    String host;
    int port;
    httpHandler->OnAskForHost(host,port);
    httpDataWritten = false;
    httpClient.connect(host.c_str(),(uint16_t)port);
}
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::sendDataToGardenbossRu()
{
  if(!httpClient.connected() || !httpHandler)
  { 
    return;
  }
    
  canCallHTTPEvent = true;
  
  delete httpData;
  httpData = new String();

  // тут посылаем данные в gardenboss.ru
  httpHandler->OnAskForData(httpData); // получили данные, которые надо отослать

  // и пишем их в клиента
  httpClient.write((uint8_t*)httpData->c_str(),httpData->length());

  delete httpData;
  httpData = new String();

}
//--------------------------------------------------------------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::OnClientConnect(CoreTransportClient& client, bool connected, int16_t errorCode)
{
  UNUSED(errorCode);
  
  #if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
  if(thingSpeakClient)
  {
    if(client == thingSpeakClient)
    {
      if(connected)
      {
            #ifdef WIFI_DEBUG
              DEBUG_LOGLN(F("ThingSpeak connected!"));
            #endif
            // успешно соединились
            sendDataToThingSpeak();        
      }
      else
      {
         #ifdef WIFI_DEBUG
          DEBUG_LOGLN(F("ThingSpeak disconnected."));
        #endif
        EnsureIoTProcessed(thingSpeakDataWritten);
        thingSpeakDataWritten = false;
      }
      
      return;
    } // if(client == thingSpeakClient)
  }
  #endif

#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
  if(httpClient)
  {
    if(client == httpClient) // клиент gardenboss.ru
    {
  
      if(connected)
      {
        httpDataWritten = false;
        sendDataToGardenbossRu();
      }
      else
      {
        if(httpDataWritten)
          EnsureHTTPProcessed(HTTP_REQUEST_COMPLETED);
        else
          EnsureHTTPProcessed(ERROR_CANT_ESTABLISH_CONNECTION);        
      }
  
      return;
    } // if(client == httpClient)
  } // if(httpClient)
#endif  


  // если мы здесь - это неизвестный клиент
  if(!connected)
  {
    // раз клиент отсоединился - нам больше не нужны его данные
    externalClientData.clear();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::processGardenbossData(uint8_t* data, size_t dataSize, bool isLastData)
{
  if(!canCallHTTPEvent || !httpHandler) // уже всё обработали
    return;

    bool enough = false;

    for(size_t i=0;i<dataSize;i++)
    {

      // тут надо бить данные по строкам, и скармливать их по одной.
      // учитывая ещё тот факт, что может быть остаток, не вместившийся в пакет.
      char ch = (char) data[i];
      
      if(ch == '\r')
        continue;
      else
      if(ch == '\n') // есть строка
      {
          #ifdef WIFI_DEBUG
            DEBUG_LOG(F("HTTP: line catched="));
            DEBUG_LOGLN(*httpData);
          #endif
          httpHandler->OnAnswerLineReceived(*httpData,enough);
          delete httpData;
          httpData = new String();
      }
      else
        *httpData += ch;
  
      if(enough)
      {
        #ifdef WIFI_DEBUG
          DEBUG_LOGLN(F("HTTP: Handler reports DONE!"));
        #endif
        canCallHTTPEvent = false;
  
        delete httpData;
        httpData = new String();
        
        break;
      } // if(enough)
    } // for

    if(isLastData)
    {
      // это последние данные
      if(canCallHTTPEvent)
      {
          #ifdef WIFI_DEBUG
            DEBUG_LOG(F("HTTP: line catched on LAST DATA="));
            DEBUG_LOGLN(*httpData);
          #endif
          
        httpHandler->OnAnswerLineReceived(*httpData,enough);
        
        if(enough)
        {
            #ifdef WIFI_DEBUG
              DEBUG_LOGLN(F("HTTP: Handler reports DONE ON LAST DATA!"));
            #endif
            canCallHTTPEvent = false;
      
        } // if(enough)
        
      } // if(canCallHTTPEvent


      delete httpData;
      httpData = new String();
      
    } // isLastData
    
}
//--------------------------------------------------------------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::OnClientDataWritten(CoreTransportClient& client, int16_t errorCode)
{
  #if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
      if(client == thingSpeakClient)
      {
        if(errorCode != CT_ERROR_NONE)
        {
          #ifdef WIFI_DEBUG
            DEBUG_LOGLN(F("ThingSpeak data write ERROR!"));
          #endif
          thingSpeakDataWritten = false;
          EnsureIoTProcessed(false);
          thingSpeakClient.disconnect();        
        }
        else
        {
          thingSpeakDataWritten = true; // выставили флаг, что записали данные нормально
        }

        return;
      } // if(client == thingSpeakClient)
  #endif

#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
  if(client == httpClient) // клиент gardenboss.ru
  {
        delete httpData;
        httpData = new String();

        if(errorCode != CT_ERROR_NONE)
        {
          #ifdef WIFI_DEBUG
            DEBUG_LOGLN(F("HTTP: can't write to gardenboss.ru!"));
          #endif
          EnsureHTTPProcessed(ERROR_HTTP_REQUEST_FAILED);
          httpClient.disconnect();        
        }
        else
        {
          // всё нормально, данные отправлены, чистим приёмный буфер, который пригодится нам для пришедших данных
          httpDataWritten = true;
        }
     return;   
  } // if(client == httpClient)
#endif  
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::ProcessUnknownClientQuery(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone)
{
  for(size_t i=0;i<dataSize;i++)
    externalClientData.push_back(data[i]);

   if(!isDone)
   {
      // ещё не вся команда пришла в клиент
      return;
   }
    
   String ctGetPrefix = F("CTGET=");
   String ctSetPrefix = F("CTSET=");

   const char* isGetFound = (const char*) MemFind(externalClientData.pData(),externalClientData.size(),ctGetPrefix.c_str(),ctGetPrefix.length());
   const char* isSetFound = (const char*) MemFind(externalClientData.pData(),externalClientData.size(),ctSetPrefix.c_str(),ctSetPrefix.length());

   if(isGetFound || isSetFound)
   {
      const char* readPtr = isGetFound ? isGetFound : isSetFound;
      const char* endOfData = (const char*) (externalClientData.pData() + externalClientData.size());

      String command;
      while(readPtr < endOfData)
      {
        if(*readPtr == '\r' || *readPtr == '\n')
          break;

        command += *readPtr;
        readPtr++;
      } // while

      // всё, нам больше не нужны данные с внешнего клиента
      externalClientData.clear();

      #ifdef WIFI_DEBUG
        DEBUG_LOG(F("ESP: incoming command are: "));
        DEBUG_LOGLN(command);
      #endif

      // теперь выполняем команду
      CommandExecuteResult fakeStream;
      CommandParser cParser;
      Command cmd;
      if(cParser.ParseCommand(command, cmd))
      {
              
        cmd.SetIncomingStream(&fakeStream); 
        MainController->ProcessModuleCommand(cmd);

        // тут выгребаем всё из буфера результатов обработки команды - и пересылаем клиенту
       client.write((uint8_t*) fakeStream.buffer.c_str(),fakeStream.buffer.length());
      
      } // if(cParser->ParseCommand(command, cmd))
    
   } // if(isGetFound || isSetFound)
   
   // просто очищаем буфер, он нам не нужен
   externalClientData.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::OnClientDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone)
{
  #if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
    if(thingSpeakClient && client == thingSpeakClient)
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("DATA AVAILABLE FOR THINGSPEAK !!!"));
      #endif
      if(isDone)
      {
        EnsureIoTProcessed(thingSpeakDataWritten);
        thingSpeakClient.disconnect();
      }

      return;
    } // if(client == thingSpeakClient)
  #endif

#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
   if(httpClient && client == httpClient) // данные для клиента gardenboss.ru
   {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("DATA AVAILABLE FOR GARDENBOSS.RU !!!"));
      #endif

      processGardenbossData(data, dataSize, isDone);
      return;
   } // if(client == httpClient)
#endif

   // если мы здесь - это данные с неизвестного клиента (например, с вебморды), обрабатываем их
   ProcessUnknownClientQuery(client, data, dataSize, isDone);
}
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::Setup()
{
  streamBuffer = new String();
  
 // сообщаем, что мы провайдер HTTP-запросов
 #ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
  httpHandler = NULL;
  httpDataWritten = false;
  httpData = new String();
  MainController->SetHTTPProvider(0,this);
  httpClient.accept(&ESP);
 #endif

#if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
     iotWriter = NULL;
     iotDone = NULL;
     thingSpeakDataWritten = false;
     IoTList.RegisterGate(this); // регистрируем себя как отсылателя данных в IoT
     thingSpeakClient.accept(&ESP);
#endif

  ESP.begin();
  ESP.subscribe(this);

  #ifdef USE_WIFI_MODULE_AS_MQTT_CLIENT
    mqtt.begin(&ESP);
  #endif  

}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::EnsureHTTPProcessed(uint16_t statusCode)
{
  if(!httpHandler) // не было флага запроса HTTP-адреса
    return;
  
  httpHandler->OnHTTPResult(statusCode); // сообщаем, что мы закончили обработку
  httpHandler = NULL;

  delete httpData;
  httpData = new String();  

}
//--------------------------------------------------------------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void WiFiModule::Update(uint16_t dt)
{ 
  UNUSED(dt);
  
  ESP.update();
  
  #ifdef USE_WIFI_MODULE_AS_MQTT_CLIENT
    mqtt.update();
  #endif // USE_WIFI_MODULE_AS_MQTT_CLIENT
  
}
//--------------------------------------------------------------------------------------------------------------------------------
bool  WiFiModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  
  PublishSingleton = NOT_SUPPORTED;

  if(command.GetType() == ctSET) // установка свойств
  {
    uint8_t argsCnt = command.GetArgsCount();
    if(argsCnt > 0)
    {
      String t = command.GetArg(0);
      if(t == WIFI_SETTINGS_COMMAND) // установить настройки вай-фай
      {
        if(argsCnt > 5)
        {
          GlobalSettings* Settings = MainController->GetSettings();
          
          int shouldConnectToRouter = atoi(command.GetArg(1));
          String routerID = command.GetArg(2);
          String routerPassword = command.GetArg(3);
          String stationID = command.GetArg(4);
          String stationPassword = command.GetArg(5);

          Settings->SetWiFiState(shouldConnectToRouter);
          Settings->SetRouterID(routerID);
          Settings->SetRouterPassword(routerPassword);
          Settings->SetStationID(stationID);
          Settings->SetStationPassword(stationPassword);
          
          if(!routerID.length())
            Settings->SetWiFiState(0); // не коннектимся к роутеру

          // убеждаемся, что отработали IoT
          #if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
            thingSpeakClient.disconnect();
            EnsureIoTProcessed(false);
          #endif

          #ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
            httpClient.disconnect();
            // убеждаемся, что мы освободили HTTP-модуль
            EnsureHTTPProcessed(ERROR_HTTP_REQUEST_CANCELLED);
          #endif

          #ifdef USE_WIFI_MODULE_AS_MQTT_CLIENT
          mqtt.reset();
          #endif

          ESP.restart(); // начинаем работу снова
                     
          PublishSingleton.Flags.Status = true;
          PublishSingleton = t; 
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
        else
          PublishSingleton = PARAMS_MISSED; // мало параметров
        
      } // WIFI_SETTINGS_COMMAND
      #ifdef USE_WIFI_MODULE_AS_MQTT_CLIENT
      else
      if(t == F("MQTT_DEL")) // удалить все топики
      {
            mqtt.DeleteAllTopics();
            PublishSingleton.Flags.Status = true;
            PublishSingleton = t; 
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;        
      } // MQTT_DEL
      else
      if(t == F("MQTT_ADD")) // добавить топик, CTSET=WIFI|MQTT_ADD|Index|Topic name|Module name|Sensor type|Sensor index|Topic type
      {
        if(argsCnt > 6)
        {
          const char* topicIndex = command.GetArg(1);
          const char* topicName = command.GetArg(2);
          const char* moduleName = command.GetArg(3);
          const char* sensorType = command.GetArg(4);
          const char* sensorIndex = command.GetArg(5);
          const char* topicType = command.GetArg(6);

          mqtt.AddTopic(topicIndex,topicName,moduleName,sensorType,sensorIndex,topicType);
          
          PublishSingleton.Flags.Status = true;
          PublishSingleton = t; 
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
        else
        {
          PublishSingleton = PARAMS_MISSED; // мало параметров
        }
        
      } // MQTT_ADD
      else
      if(t == F("MQTT")) // установить настройки MQTT: CTSET=WIFI|MQTT|Enabled flag|Server address|Server port|Interval between topics|Client id|User|Pass
      {        
        if(argsCnt > 7)
        {
            byte mqttEnabled = atoi(command.GetArg(1));
            String mqttServer = command.GetArg(2);
            String mqttPort = command.GetArg(3);
            byte mqttInterval = atoi(command.GetArg(4));
            String mqttClient = command.GetArg(5);
            String mqttUser = command.GetArg(6);
            String mqttPass = command.GetArg(7);

            if(mqttServer == "_")
              mqttServer = "";

            if(mqttPort == "_")
              mqttPort = "";

            if(mqttClient == "_")
              mqttClient = "";

            if(mqttUser == "_")
              mqttUser = "";

            if(mqttPass == "_")
              mqttPass = "";

            MemWrite(MQTT_ENABLED_FLAG_ADDRESS,mqttEnabled);
            MemWrite(MQTT_INTERVAL_BETWEEN_TOPICS_ADDRESS,mqttInterval);

            if(MainController->HasSDCard())
            {
              String mqttSettingsFileName = F("mqtt.ini");
              String newline = "\n";
              
              SdFile f;
              
              #define MQTT_WRITE_TO_FILE(f,str) f.write((const uint8_t*) str.c_str(),str.length())
              
              if(f.open(mqttSettingsFileName.c_str(), FILE_WRITE | O_TRUNC))
              {
                // адрес сервера
                MQTT_WRITE_TO_FILE(f,mqttServer);
                MQTT_WRITE_TO_FILE(f,newline);
                
                // порт
                MQTT_WRITE_TO_FILE(f,mqttPort);
                MQTT_WRITE_TO_FILE(f,newline);

                // ID клиента
                MQTT_WRITE_TO_FILE(f,mqttClient);
                MQTT_WRITE_TO_FILE(f,newline);

                // user
                MQTT_WRITE_TO_FILE(f,mqttUser);
                MQTT_WRITE_TO_FILE(f,newline);

                // pass
                MQTT_WRITE_TO_FILE(f,mqttPass);
                MQTT_WRITE_TO_FILE(f,newline);
                
                f.close();
              } // 
              
            } // if

            mqtt.reloadSettings(); // говорим клиенту, что настройки изменились
            
          
            PublishSingleton.Flags.Status = true;
            PublishSingleton = t; 
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
        else
          PublishSingleton = PARAMS_MISSED;
        
      }
      #endif // USE_WIFI_MODULE_AS_MQTT_CLIENT
    }
    else
      PublishSingleton = PARAMS_MISSED; // мало параметров
  } // SET
  else
  if(command.GetType() == ctGET) // чтение свойств
  {
    uint8_t argsCnt = command.GetArgsCount();
    if(argsCnt > 0)
    {
      String t = command.GetArg(0);
      
      if(t == IP_COMMAND) // получить данные об IP
      {
        if(!ESP.ready()) // не можем ответить на запрос немедленно
          PublishSingleton = BUSY;
        else
        {
            String staIP;
            String apIP;
            if(ESP.getIP(staIP,apIP))
            {
              PublishSingleton.Flags.Status = true;
              PublishSingleton = t; 
              PublishSingleton << PARAM_DELIMITER << apIP << PARAM_DELIMITER << staIP;
            }
            else
            {
              PublishSingleton = BUSY;
            }
          

        } // else not busy
      } // IP_COMMAND
      else
      if(t == F("PING"))
      {
        if(!ESP.ready()) // не можем ответить на запрос немедленно
          PublishSingleton = BUSY;
        else
        {
          bool pingResult = false;
          if(ESP.pingGoogle(pingResult))
          {
            String pingLine;
            
            if(pingResult)
              pingLine = F("OK");
            else
              pingResult = F("ERROR");
              
            PublishSingleton.Flags.Status = true;
            PublishSingleton = t; 
            PublishSingleton << PARAM_DELIMITER << pingLine;
          }
          else
          {
             PublishSingleton = BUSY;
          }

        } // else not busy                
      } // ping
      else
      if(t == F("MAC"))
      {
        if(!ESP.ready()) // не можем ответить на запрос немедленно
          PublishSingleton = BUSY;
        else
        {
          String staMAC, apMAC;
          if(ESP.getMAC(staMAC, apMAC))
          {
            PublishSingleton.Flags.Status = true;
            PublishSingleton = t; 
            PublishSingleton << PARAM_DELIMITER << staMAC << PARAM_DELIMITER << apMAC;            
          }
          else
          {
            PublishSingleton = BUSY;
          }
        } // else not busy        
      } // получить MAC-адрес
      else
      if(t == WIFI_SETTINGS_COMMAND)
      {
         // получить настройки Wi-Fi

        GlobalSettings* Settings = MainController->GetSettings();  

        PublishSingleton.Flags.Status = true;
        PublishSingleton = t; 
        PublishSingleton << PARAM_DELIMITER << Settings->GetWiFiState() << PARAM_DELIMITER << Settings->GetRouterID()
        << PARAM_DELIMITER << Settings->GetRouterPassword()
        << PARAM_DELIMITER << Settings->GetStationID()
        << PARAM_DELIMITER << Settings->GetStationPassword();
        
      } // WIFI_SETTINGS_COMMAND
      #ifdef USE_WIFI_MODULE_AS_MQTT_CLIENT
      else
      if(t == F("MQTT_CNT"))
      {
        // получить кол-во топиков, сохранённых на карточке
        PublishSingleton.Flags.Status = true;
        PublishSingleton = t;
        PublishSingleton << PARAM_DELIMITER << mqtt.GetSavedTopicsCount();        
        
      } // MQTT_CNT
      else
      if(t == F("MQTT_VIEW")) // посмотреть топик по индексу, CTGET=WIFI|MQTT_VIEW|Index
      {
        if(argsCnt > 1)
        {
          String topicFileName = MQTT_FILENAME_PATTERN;
          topicFileName += command.GetArg(1);
        
          // тут можем читать из файла настроек топика
                SdFile f;
                
                if(f.open(topicFileName.c_str(),FILE_READ))
                {   
                  // теперь читаем настройки топика
                  // первой строкой идёт имя топика
                  String topicName;
                  FileUtils::readLine(f,topicName);
          
                  // второй строкой - идёт имя модуля, в котором взять нужные показания
                  String moduleName;
                  FileUtils::readLine(f,moduleName);
          
                  // в третьей строке - тип датчика, числовое значение соответствует перечислению ModuleStates
                  String sensorTypeString;
                  FileUtils::readLine(f,sensorTypeString);
                            
                  // в четвёртой строке - индекс датчика в модуле
                  String sensorIndexString;
                  FileUtils::readLine(f,sensorIndexString);
          
                  // в пятой строке - тип топика: показания с датчиков (0), или статус контроллера (1).
                  // в случае статуса контроллера во второй строке - тип статуса, третья и четвёртая - зависят от типа статуса
                  String topicType;
                  FileUtils::readLine(f,topicType);
                  
                  // не забываем закрыть файл
                  f.close(); 

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = t;
                  PublishSingleton << PARAM_DELIMITER << command.GetArg(1); // index
                  PublishSingleton << PARAM_DELIMITER << topicName; // topic name
                  PublishSingleton << PARAM_DELIMITER << moduleName; // module name
                  PublishSingleton << PARAM_DELIMITER << sensorTypeString; // sensor type
                  PublishSingleton << PARAM_DELIMITER << sensorIndexString; // sensor index
                  PublishSingleton << PARAM_DELIMITER << topicType; // topic type
                  
                }
                else
                {
                  PublishSingleton = PARAMS_MISSED; // мало параметров
                }
                        
         
        }
        else
        {
          PublishSingleton = PARAMS_MISSED; // мало параметров
        }
        
      } // MQTT_VIEW
      else
      if(t == F("MQTT")) // получить настройки MQTT, CTGET=WIFI|MQTT, возвращает OK=WIFI|MQTT|Enabled flag|Server address|Server port|Interval between topics|Client id|User|Pass
      {
        if(MainController->HasSDCard())
        {
           // есть карта, читаем настройки
           byte mqttEnabled = MemRead(MQTT_ENABLED_FLAG_ADDRESS);
           if(mqttEnabled == 0xFF)
            mqttEnabled = 0;
            
           byte mqttInterval = MemRead(MQTT_INTERVAL_BETWEEN_TOPICS_ADDRESS);
           if(mqttInterval == 0xFF)
            mqttInterval = 10;

           String mqttServer, mqttPort, mqttClientId, mqttUser, mqttPass;

           String mqttSettingsFileName = F("mqtt.ini");
           SdFile f;

           if(f.open(mqttSettingsFileName.c_str(),FILE_READ))
           {
            FileUtils::readLine(f,mqttServer); // адрес сервера
            FileUtils::readLine(f,mqttPort); // порт сервера
            FileUtils::readLine(f,mqttClientId); // ID клиента
            FileUtils::readLine(f,mqttUser); // user
            FileUtils::readLine(f,mqttPass); // pass
            
            f.close();
           }
           
            // Всё прочитали, можно постить
            PublishSingleton.Flags.Status = true;
            PublishSingleton = t;
            PublishSingleton << PARAM_DELIMITER << mqttEnabled; 
            PublishSingleton << PARAM_DELIMITER << mqttServer; 
            PublishSingleton << PARAM_DELIMITER << mqttPort; 
            PublishSingleton << PARAM_DELIMITER << mqttInterval; 
            PublishSingleton << PARAM_DELIMITER << mqttClientId; 
            PublishSingleton << PARAM_DELIMITER << mqttUser; 
            PublishSingleton << PARAM_DELIMITER << mqttPass; 
           
        }
      }
      #endif // USE_WIFI_MODULE_AS_MQTT_CLIENT
    }
    else
      PublishSingleton = PARAMS_MISSED; // мало параметров
  } // GET

  MainController->Publish(this,command);

  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WIFI_MODULE

