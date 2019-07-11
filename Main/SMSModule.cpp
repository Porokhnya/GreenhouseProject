#include "SMSModule.h"
#ifdef USE_SMS_MODULE
#include "ModuleController.h"
#include "PDUClasses.h"
#include "InteropStream.h"
#if defined(USE_ALARM_DISPATCHER)
#include "AlarmDispatcher.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------
SMSModule* _thisSMSModule = NULL;
//--------------------------------------------------------------------------------------------------------------------------------
void ON_CUSD_RECEIVED(const String& cusd)
{
  if(!_thisSMSModule)
  {
    return; 
  }

  _thisSMSModule->IncomingCUSD(cusd);
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void ON_SMS_RECEIVED(const String& phoneNumber,const String& message, bool isKnownNumber)
{
  if(!_thisSMSModule)
  {
    return; 
  }

  _thisSMSModule->IncomingSMS(phoneNumber,message,isKnownNumber);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ON_INCOMING_CALL(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp)
{
  if(!_thisSMSModule)
  {
    shouldHangUp = true;
    return;
  }

  _thisSMSModule->IncomingCall(phoneNumber,isKnownNumber,shouldHangUp);
}
//--------------------------------------------------------------------------------------------------------------------------------
// функция хэширования строки
//--------------------------------------------------------------------------------------------------------------------------------
#define A_PRIME 54059 /* a prime */
#define B_PRIME 76963 /* another prime */
#define C_PRIME 86969 /* yet another prime */
//--------------------------------------------------------------------------------------------------------------------------------
unsigned int hash_str(const char* s)
{
   unsigned int h = 31 /* also prime */;
   while (*s) {
     h = (h * A_PRIME) ^ (s[0] * B_PRIME);
     s++;
   }
   return h; // or return h % C;
}
//--------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::EnsureIoTProcessed(bool success)
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
void SMSModule::sendDataToThingSpeak()
{
  thingSpeakDataWritten = false;
  
  if(!thingSpeakClient.connected() || !SIM800.ready())
  {
    EnsureIoTProcessed(false);
    return;
  }

  // тут посылаем данные в клиент для ThingSpeak
  CommandExecuteResult fakeStream;

  // резервируем нужное кол-во памяти
  fakeStream.buffer.reserve(thingSpeakDataLength + 150);

  IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();
  
  fakeStream.buffer = F("GET /update?headers=false&api_key=");
  fakeStream.buffer += iotSettings.ThingSpeakChannelID;
  fakeStream.buffer += F("&");

  // просим, чтобы записали данные в нас
  iotWriter(&fakeStream);

  fakeStream.buffer += F(" HTTP/1.1\r\nAccept: */*\r\nUser-Agent: ");
  fakeStream.buffer += IOT_USER_AGENT;
  fakeStream.buffer += F("\r\nHost: ");
  fakeStream.buffer += THINGSPEAK_HOST;
  fakeStream.buffer += F("\r\n\r\n");

  #ifdef GSM_DEBUG_MODE
    DEBUG_LOGLN(fakeStream.buffer);
  #endif

  // теперь пишем в клиент
  thingSpeakClient.write((uint8_t*) fakeStream.buffer.c_str(),fakeStream.buffer.length());

}
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::SendData(IoTService service,uint16_t dataLength, IOT_OnWriteToStream writer, IOT_OnSendDataDone onDone)
{
  thingSpeakDataWritten = false;
  
  if(!SIM800.ready())
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
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::RequestBalance()
{
      #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("Request balance..."));
      #endif

        String balanceCommand = MTS_BALANCE;
        byte op = MainController->GetSettings()->GetGSMProvider();

        switch(op)
        {
          case MTS:
          break;

          case Beeline:
            balanceCommand = BEELINE_BALANCE;
          break;

          case Megafon:
            balanceCommand = MEGAFON_BALANCE;
          break;

          case Tele2:
            balanceCommand = TELE2_BALANCE;
          break;

          case Yota:
            balanceCommand = YOTA_BALANCE;
          break;

          case MTS_Bel:
            balanceCommand = MTS_BEL_BALANCE;
          break;

          case Velcom_Bel:
            balanceCommand = VELCOM_BEL_BALANCE;
          break;

          case Privet_Bel:
            balanceCommand = PRIVET_BEL_BALANCE;
          break;
          
          case Life_Bel:
            balanceCommand = LIFE_BEL_BALANCE;
          break;

        }

    requestBalanceAsked = true;
    SIM800.sendCUSD(balanceCommand);
}
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::IncomingCUSD(const String& cusd)
{
 #ifdef GSM_DEBUG_MODE
    DEBUG_LOGLN(F("CUSD CATCHED!"));
  #endif

  if(!requestBalanceAsked)
    return;

  requestBalanceAsked = false;  

  // по CUSD мы запрашиваем только баланс, и просто пересылаем его мастеру
  String phoneNum = MainController->GetSettings()->GetSmsPhoneNumber();
  SIM800.sendSMS(phoneNum,cusd,true);  
}
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::IncomingSMS(const String& phoneNumber,const String& message, bool isKnownNumber)
{
  #ifdef GSM_DEBUG_MODE
      DEBUG_LOG(F("SIM800: INCOMING SMS RECEIVED: "));
      DEBUG_LOGLN(message);
  #endif

  if(!isKnownNumber) // неизвестный номер !!!
  {
    return;
  }

  if(message.startsWith(SMS_RESET_COMMAND))
  {
    #ifdef USE_EXTERNAL_WATCHDOG
      while(1);
    #else
      ModuleInterop.QueryCommand(ctSET,F("0|RST"),false);
    #endif
    return;
  }

  bool shouldSendSMS = false;

    // ищем команды
    int16_t idx = message.indexOf(SMS_OPEN_COMMAND); // открыть окна
    if(idx != -1)
    {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("WINDOWS->OPEN command found, execute it..."));
    #endif

        // открываем окна
        ModuleInterop.QueryCommand(ctSET,F("STATE|WINDOW|ALL|OPEN"),false);
        shouldSendSMS = true;
    }
    
    idx = message.indexOf(SMS_CLOSE_COMMAND); // закрыть окна
    if(idx != -1)
    {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("WINDOWS->CLOSE command found, execute it..."));
    #endif

      // закрываем окна
      ModuleInterop.QueryCommand(ctSET,F("STATE|WINDOW|ALL|CLOSE"),false);
      shouldSendSMS = true;
    }
    
    idx = message.indexOf(SMS_AUTOMODE_COMMAND); // перейти в автоматический режим работы
    if(idx != -1)
    {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("Automatic mode command found, execute it..."));
    #endif

      // переводим управление окнами в автоматический режим работы
      if(ModuleInterop.QueryCommand(ctSET, F("STATE|MODE|AUTO"),false))
      {
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("CTSET=STATE|MODE|AUTO command parsed, process it..."));
        #endif
    
      }

      // переводим управление поливом в автоматический режим работы
      if(ModuleInterop.QueryCommand(ctSET, F("WATER|MODE|AUTO"),false))
      {
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("CTSET=WATER|MODE|AUTO command parsed, process it..."));
        #endif
    
      }
     
      // переводим управление досветкой в актоматический режим работы    
      if(ModuleInterop.QueryCommand(ctSET, F("LIGHT|MODE|AUTO"),false))
      {
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("CTSET=LIGHT|MODE|AUTO command parsed, process it..."));
        #endif
    
      }    

      shouldSendSMS = true;
    }

    idx = message.indexOf(SMS_WATER_ON_COMMAND); // включить полив
    if(idx != -1)
    {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("Water ON command found, execute it..."));
    #endif

    // включаем полив
      if(ModuleInterop.QueryCommand(ctSET, F("WATER|ON"),false))
      {
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("CTSET=WATER|ON command parsed, process it..."));
        #endif
    
       shouldSendSMS = true;
      }
    }

    idx = message.indexOf(SMS_WATER_OFF_COMMAND); // выключить полив
    if(idx != -1)
    {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("Water OFF command found, execute it..."));
    #endif

    // выключаем полив
      if(ModuleInterop.QueryCommand(ctSET, F("WATER|OFF"),false))//,false))
      {
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("CTSET=WATER|OFF command parsed, process it..."));
        #endif
    
        shouldSendSMS = true;
      }

    }
         
    idx = message.indexOf(SMS_STAT_COMMAND); // послать статистику
    if(idx != -1)
    {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("STAT command found, execute it..."));
    #endif

      // посылаем статистику вызвавшему номеру
      SendStatToCaller(phoneNumber);

      // возвращаемся, поскольку нет необходимости посылать СМС с ответом ОК - вместо этого придёт статистика
      return;
    }

    idx = message.indexOf(SMS_BALANCE_COMMAND); // послать баланс
    if(idx != -1)
    {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("BALANCE command found, execute it..."));
    #endif

      // посылаем баланс хозяину
      RequestBalance();

      // возвращаемся, поскольку нет необходимости посылать СМС с ответом ОК - вместо этого придёт баланс
      return;
    }
    
    if(!shouldSendSMS)
    {
        // тут пробуем найти файл по хэшу переданной команды
        if(MainController->HasSDCard())
        {
          unsigned int hash = hash_str(message.c_str());         

          #ifdef GSM_DEBUG_MODE
            DEBUG_LOG(F("passed message = "));
            DEBUG_LOGLN(message);
            DEBUG_LOG(F("computed hash = "));
            DEBUG_LOGLN(String(hash));
          #endif
                        
          String filePath = F("sms");
          filePath += F("/");
          filePath += hash;
          filePath += F(".sms");
    
          SdFile smsFile;
          if(smsFile.open(filePath.c_str(),FILE_READ))
          {
      
          #ifdef GSM_DEBUG_MODE
            DEBUG_LOGLN(F("SMS file found, continue..."));
          #endif            
            // нашли такой файл, будем читать с него данные
            String answerMessage, commandToExecute;
    
            // в первой строке у нас лежит сообщение, которое надо послать после выполнения команды.
             FileUtils::readLine(smsFile,answerMessage);
             yield();
    
            // во второй строке - команда
             FileUtils::readLine(smsFile,commandToExecute);
             yield();

            // закрываем файл
            smsFile.close();

          #ifdef GSM_DEBUG_MODE
            DEBUG_LOG(F("command to execute = "));
            DEBUG_LOGLN(commandToExecute);
          #endif
          
            // парсим команду
            CommandParser* cParser = MainController->GetCommandParser();
            Command cmd;
            if(cParser->ParseCommand(commandToExecute,cmd))
            {
                #ifdef GSM_DEBUG_MODE
                  DEBUG_LOGLN(F("Command parsed, execute it..."));
                #endif                
              // команду разобрали, можно исполнять

              CommandExecuteResult fakeStream;

              cmd.SetIncomingStream(&fakeStream);
              MainController->ProcessModuleCommand(cmd);
              yield();

              // теперь получаем ответ
              if(!answerMessage.length()) 
              {
                SIM800.sendSMS(phoneNumber,fakeStream.buffer,true);
              }
              else
                SIM800.sendSMS(phoneNumber,answerMessage,true);
              
            } // if
    
            return; // возвращаемся, т.к. мы сами пошлём СМС с текстом, отличным от ОК
          } // if(smsFile)
          #ifdef GSM_DEBUG_MODE
          else
          {
            DEBUG_LOGLN(F("SMS file NOT FOUND, skip the SMS."));
          }
          #endif            
          
        } // if(MainController->HasSDCard())
        
    } // !shouldSendSMS
    

  if(shouldSendSMS) // надо послать СМС с ответом "ОК"
    SIM800.sendSMS(phoneNumber,OK_ANSWER,true);

}
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::IncomingCall(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp)
{
  #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: INCOMING CALL DETECTED!!!"));
  #endif

  shouldHangUp = true;

  if(isKnownNumber)
  {
    // отправляем статистику вызвавшему номеру
    SendStatToCaller(phoneNumber); // посылаем статистику вызвавшему
  }    
}
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::SendStatToCaller(const String& phoneNum)
{
  #ifdef GSM_DEBUG_MODE
    DEBUG_LOGLN("Try to send stat SMS to " + phoneNum + "...");
  #endif
  
  String sms; // тут будут данные для отсылания
  
  // у нас появилась возможность указывать датчики, которые мы будем отсылать в СМС.
  // поэтому тут читаем возможные настройки из файла.
  bool foundInSD = false;
  
  if(MainController->HasSDCard())
  {
    SdFile statFile;
    String fileName = F("STAT.SMS");
    if(statFile.open(fileName.c_str(),FILE_READ))
    {
      foundInSD = true;
      String module1,sensor1,label1,module2,sensor2,label2;
      
      FileUtils::readLine(statFile,module1);
      yield();
      FileUtils::readLine(statFile,sensor1);
      yield();
      FileUtils::readLine(statFile,label1);
      yield();
      FileUtils::readLine(statFile,module2);
      yield();
      FileUtils::readLine(statFile,sensor2);
      yield();
      FileUtils::readLine(statFile,label2);
      yield();

      // читаем с первого модуля
      int currModuleIndex = module1.toInt();
      if(currModuleIndex > 0)
      {
        // есть указание индекса модуля, транслируем его в имя известного модуля
        const char* knownModule = GetKnownModuleName(currModuleIndex);
        if(knownModule)
          sms += RequestDataFromKnownModule(knownModule, currModuleIndex, sensor1.toInt(), label1);
        
      } // if

      // читаем со второго модуля
      currModuleIndex = module2.toInt();
      if(currModuleIndex > 0)
      {
        // есть указание индекса модуля, транслируем его в имя известного модуля
        const char* knownModule = GetKnownModuleName(currModuleIndex);
        if(knownModule)
          sms += RequestDataFromKnownModule(knownModule, currModuleIndex, sensor2.toInt(), label2);
        
      } // if

      statFile.close();
    } // if(statFile)
     
  } // if(MainController->HasSDCard())
  
  if(!foundInSD)
  {
    // тут ничего не найдено на SD, применяем настройки по умолчанию
    const char* knownModule = GetKnownModuleName(STATUS_SMS_DEFAULT_MODULE1);
    if(knownModule)
      sms += RequestDataFromKnownModule(knownModule, STATUS_SMS_DEFAULT_MODULE1, STATUS_SMS_DEFAULT_SENSOR1, STATUS_SMS_DEFAULT_LABEL1);

    knownModule = GetKnownModuleName(STATUS_SMS_DEFAULT_MODULE2);
    if(knownModule)
      sms += RequestDataFromKnownModule(knownModule, STATUS_SMS_DEFAULT_MODULE2, STATUS_SMS_DEFAULT_SENSOR2, STATUS_SMS_DEFAULT_LABEL2);
      
  } // if(!foundInSD)


#if defined(USE_TEMP_SENSORS) && defined(SEND_WINDOW_STATE_IN_STATUS_SMS)

  // тут получаем состояние окон
  if(ModuleInterop.QueryCommand(ctGET,F("STATE|WINDOW|ALL"),true))
  {
    yield();

    sms += W_STATE;

    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("Command CTGET=STATE|WINDOW|ALL parsed, execute it..."));
    #endif

    const char* strPtr = PublishSingleton.Text.c_str();
     if(strstr_P(strPtr,(const char*) STATE_OPEN))
        sms += W_OPEN;
      else
        sms += W_CLOSED;


     sms += NEWLINE;
 
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOG(F("Receive answer from STATE: ")); 
      DEBUG_LOGLN(PublishSingleton.Text);
    #endif
  }
 #endif

 #if defined(USE_WATERING_MODULE) && defined(SEND_WATER_STATE_IN_STATUS_SMS)
    // получаем состояние полива
  if(ModuleInterop.QueryCommand(ctGET,F("WATER"),true))
  {
    sms += WTR_STATE;

    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("Command CTGET=WATER parsed, execute it..."));
    #endif

    const char* strPtr = PublishSingleton.Text.c_str();
    if(strstr_P(strPtr,(const char*) STATE_OFF))
      sms += WTR_OFF;
    else
      sms += WTR_ON;
          
  }
  #endif

  // тут отсылаем SMS
  if(sms.length())
    SIM800.sendSMS(phoneNum,sms,true);

}
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::Setup()
{
  _thisSMSModule = this;
  
 // сообщаем, что мы провайдер HTTP-запросов
 #ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
 httpHandler = NULL;
  httpDataWritten = false;
  httpData = new String();
  MainController->SetHTTPProvider(1,this);
  httpClient.accept(&SIM800); 
 #endif


  #if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
     iotWriter = NULL;
     iotDone = NULL;
     thingSpeakDataWritten = false;
     IoTList.RegisterGate(this); // регистрируем себя как отсылателя данных в IoT
     thingSpeakClient.accept(&SIM800);     
  #endif  

    #if defined(USE_ALARM_DISPATCHER) && defined(CLEAR_ALARM_STATUS)
      processedAlarmsClearTimer = millis();
    #endif

  SIM800.begin();
  SIM800.subscribe(this);
   
  // настройка модуля тут
}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
//--------------------------------------------------------------------------------------------------------------------------------
bool SMSModule::CanMakeQuery() // тестирует, может ли модуль сейчас сделать запрос
{
  return SIM800.ready();
}
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::MakeQuery(HTTPRequestHandler* handler) // начинаем запрос по HTTP
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
void SMSModule::sendDataToGardenbossRu()
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
void SMSModule::OnClientConnect(CoreTransportClient& client, bool connected, int16_t errorCode)
{
  UNUSED(errorCode);
  
  #if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
  if(thingSpeakClient)
  {
    if(client == thingSpeakClient)
    {
      if(connected)
      {
            #ifdef GSM_DEBUG_MODE
              DEBUG_LOGLN(F("ThingSpeak connected!"));
            #endif
            // успешно соединились
            sendDataToThingSpeak();        
      }
      else
      {
         #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("ThingSpeak disconnected."));
        #endif
        EnsureIoTProcessed(thingSpeakDataWritten);
        thingSpeakDataWritten = false;
      }
      
      return;
    } // if(client == thingSpeakClient)
  }
  #endif

#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
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
}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::processGardenbossData(uint8_t* data, size_t dataSize, bool isLastData)
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
          #ifdef GSM_DEBUG_MODE
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
        #ifdef GSM_DEBUG_MODE
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
          #ifdef GSM_DEBUG_MODE
            DEBUG_LOG(F("HTTP: line catched on LAST DATA="));
            DEBUG_LOGLN(*httpData);
          #endif
          
        httpHandler->OnAnswerLineReceived(*httpData,enough);
        
        if(enough)
        {
            #ifdef GSM_DEBUG_MODE
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
void SMSModule::OnClientDataWritten(CoreTransportClient& client, int16_t errorCode)
{
  #if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
      if(client == thingSpeakClient)
      {
        if(errorCode != CT_ERROR_NONE)
        {
          #ifdef GSM_DEBUG_MODE
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

#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
  if(client == httpClient) // клиент gardenboss.ru
  {
        delete httpData;
        httpData = new String();

        if(errorCode != CT_ERROR_NONE)
        {
          #ifdef GSM_DEBUG_MODE
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
void SMSModule::OnClientDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone)
{
  #if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
    if(thingSpeakClient && client == thingSpeakClient)
    {
      #ifdef GSM_DEBUG_MODE
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

#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
   if(httpClient && client == httpClient) // данные для клиента gardenboss.ru
   {
      #ifdef GSM_DEBUG_MODE
        DEBUG_LOGLN(F("DATA AVAILABLE FOR GARDENBOSS.RU !!!"));
        /*
        DEBUG_LOGLN(F("DATA IS: "));
        for(size_t i=0;i<dataSize;i++)
        {
          Serial.print((char)data[i]);
        }
        Serial.println();
        */
      #endif

      processGardenbossData(data, dataSize, isDone);
      return;
   } // if(client == httpClient)
#endif

}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
//--------------------------------------------------------------------------------------------------------------------------------
void SMSModule::EnsureHTTPProcessed(uint16_t statusCode)
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
void SMSModule::Update(uint16_t dt)
{ 

  UNUSED(dt);
  
  SIM800.update();

  #ifdef SEND_WORK_STARTED_SMS
    static bool workStartedSmsSent = false;
    if(SIM800.ready())
    {
        if(!workStartedSmsSent)
        {
          workStartedSmsSent = true;
          SIM800.sendSMS(MainController->GetSettings()->GetSmsPhoneNumber(), WORK_STARTED_SMS_TEXT,false);
        }
    }
  #endif

    #if defined(USE_ALARM_DISPATCHER) && defined(CLEAR_ALARM_STATUS)
    
      unsigned long curAlarmsTimer = millis();
      unsigned long wantedAlarmsClearInterval = ALARM_CLEAR_INTERVAL*60000;

      if((curAlarmsTimer - processedAlarmsClearTimer) > wantedAlarmsClearInterval)
      {
        // настало время очистить сработавшие тревоги
        processedAlarmsClearTimer = curAlarmsTimer;
        AlarmDispatcher* alD = MainController->GetAlarmDispatcher();
        if(alD)
          alD->ClearProcessedAlarms();
        
      }
     

    #endif // #if defined(USE_ALARM_DISPATCHER) && defined(CLEAR_ALARM_STATUS)


    #if defined(USE_ALARM_DISPATCHER)

      static uint32_t alarmTimer = 0;

      if(millis() - alarmTimer > 10000) // незачем чаще чем раз в 10 секунд проверять тревоги
      {
        alarmTimer = millis();
        
        if(SIM800.ready())
        {        
          // проверяем, есть ли для нас тревоги
          AlarmDispatcher* alD = MainController->GetAlarmDispatcher();
          if(alD->HasSMSAlarm())
          {
            #ifdef GSM_DEBUG_MODE
              DEBUG_LOGLN(F("HAS ALARM VIA SMS, send it..."));
            #endif

            // имеем тревогу, которую надо послать по СМС
            String dt = alD->GetSMSAlarmData();
            alD->MarkSMSAlarmDone();
            SIM800.sendSMS(MainController->GetSettings()->GetSmsPhoneNumber(), dt,false);
          }
        } // if(SIM800.ready())
      } // if
      
   #endif // #if defined(USE_ALARM_DISPATCHER)
     

}
//--------------------------------------------------------------------------------------------------------------------------------
const char* SMSModule::GetKnownModuleName(int moduleIndex)
{
  // индексы модулей транслируются в имена так:
  
  // 1 - модуль температур
  // 2 - модуль влажности
  // 3 - модуль освещённости
  // 4 - модуль влажности почвы
  
  switch(moduleIndex)
  {
    case 1: return "STATE";
    case 2: return "HUMIDITY";
    case 3: return "LIGHT";
    case 4: return "SOIL";
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------
String SMSModule::RequestDataFromKnownModule(const char* knownModule, int moduleIndex, int sensorIndex, const String& label)
{
  String result;
  
  // индексы модулей кодируются так:
  // 1 - модуль температур
  // 2 - модуль влажности
  // 3 - модуль освещённости
  // 4 - модуль влажности почвы
    
  ModuleStates st = StateUnknown;
  ModuleStates st2 = StateUnknown;
  
  switch(moduleIndex)
  {
    case 1: st = StateTemperature; break;
    case 2: { st = StateTemperature; st2 = StateHumidity; } break;
    case 3: st = StateLuminosity; break;
    case 4: st = StateSoilMoisture; break;
  }

  AbstractModule* neededModule = MainController->GetModuleByID(knownModule);
  
  if(!neededModule || st == StateUnknown) // не нашли модуль, или неизвестное состояние было запрошено
    return result;

  OneState* os = neededModule->State.GetState(st,sensorIndex);
  OneState* os2 = NULL;
  
  if(os) // есть состояние, например, температура
  {
    result += label; // выводим метку
    result += ": ";

    if(os->HasData()) // есть данные, выводим их
    {
      result += *os; // данные
      result += os->GetUnit(); // и вид показаний с датчика
    }
    else // нет данных с датчика
      result += NO_DATA;
      
  } // if(os)
  
  if(st2 != StateUnknown) // для влажности надо получить ещё и влажность, помимо температуры, т.к. датчик - с двумя типами показаний
  {
    os2 = neededModule->State.GetState(st2,sensorIndex);
    
    if(os2) // есть состояние, например, влажность
    {
      if(os) // если было первое состояние, например, температура, то перед текущим показанием ставим слеш, разделяя таким образом данные
        result += "/";

      if(os2->HasData()) // есть данные, можно выводить
      {
        result += *os2; // выводим показания
        result += os2->GetUnit(); // и их вид
      }
      else // нет данных
       result += NO_DATA;
      
    } // if(os2)
    
  } // if(st2 != StateUnknown)

  if(os || os2) // если хотя бы что-то получили - завершаем строку
    result += NEWLINE;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------
bool  SMSModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);

  size_t argsCount = command.GetArgsCount();
  GlobalSettings* Settings = MainController->GetSettings();
  
  if(command.GetType() == ctSET) 
  {
    if(!argsCount) // нет аргументов
    {
      PublishSingleton = PARAMS_MISSED;
    }
    else
    {
      String t = command.GetArg(0);
      if(t == F("ADD"))
      {
        if(argsCount < 4)
        {
          PublishSingleton = PARAMS_MISSED;
        }
        else
        {
            if(MainController->HasSDCard())
            {
              
              // добавить кастомное СМС

              // получаем закодированное в HEX сообщение
              const char* hexMessage = command.GetArg(1);
              String message;

              // переводим его в UTF-8
              while(*hexMessage)
              {
                message += (char) WorkStatus::FromHex(hexMessage);
                hexMessage += 2;
              }

              // получаем его хэш
              unsigned int hash = hash_str(message.c_str());

              #ifdef GSM_DEBUG_MODE
                DEBUG_LOG(F("passed message = "));
                DEBUG_LOGLN(message);
                DEBUG_LOG(F("computed hash = "));
                DEBUG_LOGLN(String(hash));
              #endif
              // создаём имя файлв
              String filePath = F("sms");
              SDFat.mkdir(filePath.c_str());
              filePath += F("/");
              filePath += hash;
              filePath += F(".sms");

              SdFile smsFile;
              if(smsFile.open(filePath.c_str(),FILE_WRITE | O_TRUNC))
              {
                // в аргументе номер 2 у нас лежит ответ, который надо послать
                hexMessage = command.GetArg(2);
                message = "";
    
                  // переводим его в UTF-8
                  while(*hexMessage)
                  {
                    message += (char) WorkStatus::FromHex(hexMessage);
                    hexMessage += 2;
                    yield();
                  }

                // пишем первой строчкой ответ, который надо послать
                smsFile.print(message.c_str());
                yield();
                smsFile.println();
                yield();
                
                // теперь пишем команду, которую надо выполнить
                for(uint8_t i=3;i<argsCount;i++)
                {
                  const char* arg = command.GetArg(i);
                  smsFile.print(arg);
                  if(i < (argsCount-1))
                    smsFile.write('|');

                  yield();
                } // for
                
                smsFile.println();
                yield();
                // закрываем файл
                smsFile.close();
              } // if(smsFile)

    
              PublishSingleton = REG_SUCC;
              PublishSingleton.Flags.Status = true;
              
            } // if(MainController->HasSDCard())
            else
              PublishSingleton = NOT_SUPPORTED;
        } // else
        
      } // ADD
      else if(t == F("STATSENSORS"))
      {
        // запросили установку датчиков СМС статистики
        // приходит: CTSET=SMS|STATSENSORS|Module1|Sensor1|label1|Module2|Sensor2|Label2
        if(argsCount < 7)
        {
          PublishSingleton = t;
          PublishSingleton << PARAM_DELIMITER;
          PublishSingleton << PARAMS_MISSED;
        }
        else
        {
            if(MainController->HasSDCard())
            {
                SdFile statFile;
                String fileName = F("STAT.SMS");
                if(statFile.open(fileName.c_str(),FILE_WRITE | O_TRUNC))
                {
                  // пишем в файл параметры, не забывая расшифровывать подпись из HEX в UTF-8
                  statFile.println(command.GetArg(1)); // пишем модуль 1
                  yield();
                  statFile.println(command.GetArg(2)); // пишем датчик 1
                  yield();

                  const char* hexLabel = command.GetArg(3);
                  String label;
    
                  // переводим подпись первого датчика в UTF-8
                  if(strcmp(hexLabel,"_")) // если имя датчика не пустое (строка "_") - тогда надо раскодировать
                  {
                    while(*hexLabel)
                    {
                      label += (char) WorkStatus::FromHex(hexLabel);
                      hexLabel += 2;
                      yield();
                    }
                  }
                  else
                    label = hexLabel;
                    
                  statFile.println(label.c_str());
                  yield();

                  // теперь читаем данные для второго датчика
                  statFile.println(command.GetArg(4)); // пишем модуль 2
                  yield();
                  statFile.println(command.GetArg(5)); // пишем датчик 2

                  hexLabel = command.GetArg(6);
                  label = "";
    
                  // переводим подпись второго датчика в UTF-8
                  if(strcmp(hexLabel,"_")) // если имя датчика не пустое (строка "_") - тогда надо раскодировать
                  {
                    while(*hexLabel)
                    {
                      label += (char) WorkStatus::FromHex(hexLabel);
                      hexLabel += 2;
                      yield();
                    }
                  }
                  else
                    label = hexLabel;

                  statFile.println(label.c_str());
                  yield();

                  // всё записали, можно закрывать файл
                  
                  statFile.close();
                }
            }
          
            PublishSingleton.Flags.Status = true;
            PublishSingleton = t;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;                  
        }
        
      }
      else if(t == F("PROV"))
      {
        // запросили установить провайдера GSM
        if(argsCount < 2)
        {
          PublishSingleton = t;
          PublishSingleton << PARAM_DELIMITER;
          PublishSingleton << PARAMS_MISSED;
        }
        else
        {
          String helper = command.GetArg(1);
          byte p = (byte) helper.toInt();
          GlobalSettings* s = MainController->GetSettings();
          if(s->SetGSMProvider(p))
          {
            PublishSingleton.Flags.Status = true;
            PublishSingleton = t;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
          else
          {
            
            PublishSingleton = t;
            PublishSingleton << PARAM_DELIMITER << PARAMS_MISSED;
          }
                    
        } // enough args
        
        
      }
      else
        PublishSingleton = UNKNOWN_COMMAND;
      
    } // else have args
  }
  else
  if(command.GetType() == ctGET) //получить статистику
  {

    if(!argsCount) // нет аргументов
    {
      PublishSingleton = PARAMS_MISSED;
    }
    else
    {
      String t = command.GetArg(0);

        if(t == STAT_COMMAND) // запросили данные статистики
        {
          SendStatToCaller(Settings->GetSmsPhoneNumber()); // посылаем статистику на указанный номер телефона
        
          PublishSingleton.Flags.Status = true;
          PublishSingleton = STAT_COMMAND; 
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
        #ifdef GSM_DEBUG_MODE
        else if(t == F("DUMP"))
        {
          SIM800.dumpReceiveBuffer();
          PublishSingleton.Flags.Status = true;
          PublishSingleton = t;
        }
        #endif
        else if(t == F("PROV")) // запросили провайдера GSM
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = t;
          PublishSingleton << PARAM_DELIMITER;
          PublishSingleton << MainController->GetSettings()->GetGSMProvider();
        }
        else if(t == F("STATSENSORS"))
        {
           // получить привязку датчиков к СМС статистики
           // Отсылаем: OK=SMS|STATSENSORS|Module1|Sensor1|label1|Module2|Sensor2|Label2
           PublishSingleton.Flags.Status = true;
           PublishSingleton = t;
           PublishSingleton << PARAM_DELIMITER;
           
           String defCommand = F("0|0|_|0|0|_");
           
            if(MainController->HasSDCard())
            {
              SdFile statFile;
              String fName = F("STAT.SMS");
              if(statFile.open(fName.c_str() ,FILE_READ))
              {
                String module1, sensor1, label1, module2, sensor2, label2;
                
                FileUtils::readLine(statFile,module1);
                yield();
                FileUtils::readLine(statFile,sensor1);
                yield();
                FileUtils::readLine(statFile,label1);
                yield();
                FileUtils::readLine(statFile,module2);
                yield();
                FileUtils::readLine(statFile,sensor2);
                yield();
                FileUtils::readLine(statFile,label2);
                yield();
  
                if(!module1.length())
                  module1 = "0";
                  
                if(!sensor1.length())
                  sensor1 = "0";
                  
                if(!label1.length())
                  label1 = "_";
  
                if(!module2.length())
                  module2 = "0";
                  
                if(!sensor2.length())
                  sensor2 = "0";
                  
                if(!label2.length())
                  label2 = "_";
                  
                PublishSingleton << module1 << PARAM_DELIMITER << sensor1 << PARAM_DELIMITER;
  
                  if(label1 == "_")
                  {
                    PublishSingleton << label1;
                  }
                  else
                  {
                    const char* str = label1.c_str();
                    while(*str)
                    {
                      PublishSingleton << WorkStatus::ToHex(*str++);
                    }
                  }
  
                  PublishSingleton << PARAM_DELIMITER;
  
                  PublishSingleton << module2 << PARAM_DELIMITER << sensor2 << PARAM_DELIMITER;
  
                  if(label2 == "_")
                  {
                    PublishSingleton << label2;
                  }
                  else
                  {
                    const char* str = label2.c_str();
                    while(*str)
                    {
                      PublishSingleton << WorkStatus::ToHex(*str++);
                    }
                  }           
                
                statFile.close();
              }
              else // не удалось открыть файл
              {
                PublishSingleton << defCommand;
              }
            } // if(MainController->HasSDCard())
            else
            {
              PublishSingleton << defCommand;
            }
        }
        else if(t == BALANCE_COMMAND) 
        { // получить баланс

          RequestBalance();
          
          PublishSingleton.Flags.Status = true;
          PublishSingleton = BALANCE_COMMAND; 
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          
        }
        else
        {
          // неизвестная команда
          PublishSingleton = UNKNOWN_COMMAND;
        } // else
    } // else have arguments
    
  } // if
 
 // отвечаем на команду
    MainController->Publish(this,command);
    
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------
#endif // #ifdef USE_SMS_MODULE

