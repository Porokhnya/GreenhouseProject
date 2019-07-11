#include "IoTModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_IOT_MODULE)
IoTModule* _thisIotModule;
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_IOT_MODULE)
void iotWrite(Stream* writeTo) // вызывается, когда в поток можно писать, в этом обработчике можно писать в поток данные длиной dataLength
{
  _thisIotModule->Write(writeTo);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void iotDone(const IoTCallResult& result)
{
_thisIotModule->Done(result);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IoTModule::Write(Stream* writeTo)
{
  if(!writeTo)
    return;

#ifdef IOT_DEBUG
  DEBUG_LOGLN(F("Write IOT DATA TO STREAM..."));
#endif    

  writeTo->write(dataToSend->c_str(),dataToSend->length());
  delete dataToSend;
  dataToSend = new String();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
AbstractModule* IoTModule::FindModule(byte index)
{
  switch(index)
  {
    case 1:
      return MainController->GetModuleByID("STATE"); // температурные датчики
    case 2:
      return MainController->GetModuleByID("HUMIDITY"); // датчики влажности
    case 3:
      return MainController->GetModuleByID("LIGHT"); // датчики освещённости
    case 4:
      return MainController->GetModuleByID("SOIL"); // датчики влажности почвы
    case 5:
      return MainController->GetModuleByID("PH"); // датчики pH
      
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IoTModule::SwitchToWaitMode()
{
#ifdef IOT_DEBUG
  DEBUG_LOGLN(F("IOT - switch to wait mode..."));
#endif    
  
     delete dataToSend;
     dataToSend = new String();
     inSendData = false;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IoTModule::CollectDataForThingSpeak()
{
#ifdef IOT_DEBUG
  DEBUG_LOGLN(F("IOT - collect data for ThingSpeak..."));
#endif    
  
  // тут собираем данные для ThingSpeak, в понятном ему формате
  // Следует учесть один момент: поскольку у нас датчики привязаны к полям канала ThingSpeak,
  // мы должны ВСЕГДА итерировать индекс поля канала!
  
  delete dataToSend;
  dataToSend = new String();

  IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();
  for(byte i=0;i<8;i++) // максимум 8 датчиков на канал
  {
      AbstractModule* mod = FindModule(iotSettings.Sensors[i].ModuleID);
      if(!mod) // не нашли связанный модуль
        continue;

     OneState* os = mod->State.GetState((ModuleStates)iotSettings.Sensors[i].Type,iotSettings.Sensors[i].SensorIndex);
     if(!os) // не нашли датчик с переданным индексом и нужного типа
      continue;

      if(os->HasData())
      {
        // с датчика есть показания, можно формировать данные
        if(dataToSend->length())
          *dataToSend += F("&");
          
        *dataToSend += F("field");
        byte fieldIndex = i+1; // индекс поля канала ThingSpeak начинается с 1, поэтому добавляем 1
        *dataToSend += String(fieldIndex);
        *dataToSend += F("=");

         String  sensorData = *os;

        // ThingSpeak просит float с точкой, поэтому заменяем запятую на точку
        sensorData.replace(',','.');
        
        *dataToSend += sensorData;
      }    
  } // for

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IoTModule::SwitchToNextService()
{
#ifdef IOT_DEBUG
  DEBUG_LOGLN(F("IOT - switch to next service..."));
#endif    

  
  if(!services.size()) // ничего нету для работы, переключаемся в режим ожидания
  {
    SwitchToWaitMode();
    return;
  }
  
   currentService = services[services.size() - 1];
   services.pop();
   currentGateIndex = -1;

  //ТУТ СОБИРАЕМ ДАННЫЕ в формате для сервиса!!!
   switch(currentService)
   {
     case iotThingSpeak:
        CollectDataForThingSpeak();
     break;
   }

   
   ProcessNextGate(); // обрабатываем следующий шлюз, уже с новым сервисом 
}

//--------------------------------------------------------------------------------------------------------------------------------------
void IoTModule::Done(const IoTCallResult& result)
{
  // выводим тестовые данные
#ifdef IOT_DEBUG
  DEBUG_LOG(F("IOT done, success: "));
  DEBUG_LOGLN(result.success ? F("true") : F("false") );
#endif    

  delete dataToSend;
  dataToSend = new String();

  // проверяем результат отработки отсыла данных через переданный шлюз
  if(result.success) 
  {
     // данные отосланы успешно, можно переходить на следующий сервис, ибо нет нужды пихать одни и те же данные на один и тот же сервис через разные шлюзы
     SwitchToNextService();
  }
  else
  {
    ProcessNextGate(); // вызов неуспешен, переходим на следующий шлюз
  }

  
}

#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void IoTModule::Setup()
{
 #if defined(USE_IOT_MODULE) 
 
  _thisIotModule = this;
 
  dataToSend = new String();
  updateTimer = 0;
  inSendData = false;
 #endif 
 
  // настройка модуля тут

 }
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_IOT_MODULE) 
 void IoTModule::SendDataToIoT()
 {
  if(inSendData) // уже что-то посылаем
    return;

#ifdef IOT_DEBUG
  DEBUG_LOGLN(F("IOT - send data to IoT..."));
#endif   

 IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();

  services.clear();
  
  if(iotSettings.Flags.ThingSpeakEnabled) // ThingSpeak включен
      services.push_back(iotThingSpeak);
      
  //TODO: СЮДА ДОБАВЛЯЕМ ПОДДЕРЖИВАЕМЫЕ СЕРВИСЫ


  // говорим, что мы в процессе обработки данных
  inSendData = true;

  SwitchToNextService(); // начинаем обработку первого сервиса
  
 }
//--------------------------------------------------------------------------------------------------------------------------------------
 void IoTModule::ProcessNextGate()
 {
#ifdef IOT_DEBUG
  DEBUG_LOGLN(F("IOT - switch to next gate..."));
#endif
     
    currentGateIndex++; // переходим на следующий шлюз
    
    // тут получаем следующий шлюз в списке шлюзов
    IoTGate* gate = IoTList.GetGate(currentGateIndex);

    if(!gate) 
    {
       // мы закончили обрабатывать только один сервис, надо перейти к следующему
        SwitchToNextService();
        return;
    } // !gate

#ifdef IOT_DEBUG
  DEBUG_LOGLN(F("IOT - ask gate for send data..."));
#endif   

    // тут можем обрабатывать отсыл данных через выбранный шлюз
    gate->SendData(currentService,dataToSend->length(), iotWrite, iotDone);    
 }

 #endif

//--------------------------------------------------------------------------------------------------------------------------------------
void IoTModule::Update(uint16_t dt)
{ 
 #ifdef USE_IOT_MODULE  
  if(inSendData)
    return;

  IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();
  bool canWork =   iotSettings.Flags.ThingSpeakEnabled;

  if(canWork && iotSettings.UpdateInterval > 0)
  {
  // обновление модуля тут
    updateTimer += dt;
    if(updateTimer > iotSettings.UpdateInterval) 
    {
      updateTimer = 0;
      #ifdef IOT_DEBUG
        DEBUG_LOGLN(F("IOT - want to send data..."));
      #endif         
      SendDataToIoT();
    }
  }
#else
  UNUSED(dt);  
#endif
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool IoTModule::ExecCommand(const Command& command, bool wantAnswer)
{
 if(wantAnswer) 
    PublishSingleton = NOT_SUPPORTED;
    
  if(command.GetType() == ctSET) // установка свойств
  {

    uint8_t argsCnt = command.GetArgsCount();
    if(argsCnt < 1)
    {
      if(wantAnswer) 
        PublishSingleton = PARAMS_MISSED; // не хватает параметров
      
    } // argsCnt < 1
    else
    {
          String param = command.GetArg(0);
          if(param == F("T_SETT")) // запросили установить настройки IoT: CTSET=IOT|T_SETT
          {
              if(argsCnt < 28)
              {
                PublishSingleton = PARAMS_MISSED;
              }
              else
              {
                // с параметрами норм, разбираем
                PublishSingleton.Flags.Status = true;
                PublishSingleton = REG_SUCC;
                
                IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();

                byte iter = 1;
                
                String strHelper = command.GetArg(iter++);
                byte fl = (byte) strHelper.toInt();
                memcpy(&(iotSettings.Flags),&fl,sizeof(byte));

                strHelper = command.GetArg(iter++);

                iotSettings.UpdateInterval = (unsigned long) strHelper.toInt();

                // теперь выцепляем настройки датчиков
                for(byte i=0;i<8;i++)
                {
                    strHelper = command.GetArg(iter++);
                    iotSettings.Sensors[i].ModuleID = (byte) strHelper.toInt();

                    strHelper = command.GetArg(iter++);
                    iotSettings.Sensors[i].Type = (byte) strHelper.toInt();

                    strHelper = command.GetArg(iter++);
                    iotSettings.Sensors[i].SensorIndex = (byte) strHelper.toInt();
                } // for

                // теперь получаем ID канала thingspeak
                strncpy(iotSettings.ThingSpeakChannelID,command.GetArg(iter++),sizeof(iotSettings.ThingSpeakChannelID)-1);


                MainController->GetSettings()->SetIoTSettings(iotSettings);
              } // else
            
          } // if(param == F("T_SETT")) // T_SETT
          
    } // else
    
  } // ctSET    
  else
  if(command.GetType() == ctGET) // запрос свойств
  {
      uint8_t argsCnt = command.GetArgsCount();
      if(argsCnt < 1)
      {
        if(wantAnswer) 
          PublishSingleton = PARAMS_MISSED; // не хватает параметров
        
      } // argsCnt < 1 
      else
      {     
        String param = command.GetArg(0);
        
        if(param == F("T_SETT")) // запросили настройки IoT: CTGET=IOT|T_SETT
        {
          PublishSingleton.Flags.Status = true;

          IoTSettings iotSettings = MainController->GetSettings()->GetIoTSettings();

          byte fl = 0;
          memcpy(&fl,&(iotSettings.Flags),sizeof(byte));
          PublishSingleton = fl;
          PublishSingleton << PARAM_DELIMITER;
          PublishSingleton << iotSettings.UpdateInterval;

          // теперь выводим настройки датчиков, поскольку настройки каналов лучше делать после датчиков (например, введение поддержки народного мониторинга)
          for(byte i=0;i<8;i++)
          {
            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << iotSettings.Sensors[i].ModuleID;     

            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << iotSettings.Sensors[i].Type;     

            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << iotSettings.Sensors[i].SensorIndex;     
          }

          PublishSingleton << PARAM_DELIMITER;
          PublishSingleton << iotSettings.ThingSpeakChannelID;
                
        } // param == F("T_SETT")
        
      } // else
  }
  
  MainController->Publish(this,command); 

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------

