#include "ReservationModule.h"
#include "ModuleController.h"
#include "Memory.h"
//--------------------------------------------------------------------------------------------------------------------------------------
void ReservationModule::Setup()
{
  // настройка модуля тут
  MainController->SetReservationResolver(this);

}
//--------------------------------------------------------------------------------------------------------------------------------------
void ReservationModule::LoadReservations()
{
  ClearReservations();

  uint16_t addr = RESERVATION_ADDR;
  
  uint8_t header1, header2;
  header1 = MemRead(addr++);
  header2 = MemRead(addr++);

  if(!(header1 == SETT_HEADER1 && header2 == SETT_HEADER2)) // ничего не сохранено
    return;

  // читаем кол-во записей
  uint8_t cnt = MemRead(addr++);

  // теперь читаем все записи
  for(uint8_t i=0;i<cnt;i++)
  {
      ReservationRecord* rec = new ReservationRecord;
      rec->Type = MemRead(addr++);
      uint8_t sensorsCount = MemRead(addr++);

      for(uint8_t j=0;j<sensorsCount;j++)
      {
        ReservationItem ri;
        uint8_t raw = MemRead(addr++);
        memcpy(&ri,&raw,sizeof(uint8_t));
        
        rec->Items.push_back(ri);
      } // for

      records.push_back(rec);
    
  } // for

}
//--------------------------------------------------------------------------------------------------------------------------------------
void ReservationModule::ClearReservations()
{
  for(size_t i=0;i<records.size();i++)
  {
    ReservationRecord* rec = records[i];
    delete rec;
  }

  records.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ReservationModule::SaveReservations()
{
  uint16_t addr = RESERVATION_ADDR;

  // пишем заголовок
  MemWrite(addr++,SETT_HEADER1);
  MemWrite(addr++,SETT_HEADER2);

  // пишем кол-во записей
  uint8_t cnt = records.size();
  MemWrite(addr++,cnt);

  // теперь пишем записи
  for(uint8_t i=0;i<cnt;i++)
  {
    ReservationRecord* rec = records[i];

    // пишем тип записи
    MemWrite(addr++,rec->Type);

    // пишем кол-во датчиков, входящих в список резервирования
    uint8_t sensorsCnt = rec->Items.size();
    MemWrite(addr++,sensorsCnt);

    // пишем все датчики
    for(uint8_t j=0;j<sensorsCnt;j++)
    {
      ReservationItem it = rec->Items[j];
      uint8_t raw;
      memcpy(&raw,&it,sizeof(uint8_t));
      MemWrite(addr++,raw);
    } // for
  } // for
  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// возвращает первое попавшееся состояние с данными, основываясь на списках резервирования для указанного типа
// датчиков. Параметр sourceModule - содержит модуль с датчиком, с которого нет показаний.
// В списках резервирования ищется датчик, привязанный к этому модулю и с индексом sensorIndex, соответствующий
// типу sensorType. Если такой датчик найден, то из списка резервирования возвращается первое состояние,
// для которого есть данные.
//--------------------------------------------------------------------------------------------------------------------------------------
OneState* ReservationModule::GetReservedState(AbstractModule* sourceModule, ModuleStates sensorType, uint8_t sensorIndex)
{
  // пробегаемся по всем спискам
  for(size_t i=0;i<records.size();i++)
  {
    ReservationRecord* rec = records[i];

    // смотрим, наш ли тип резервирования?
    bool isOurReservationType = false;
    switch(rec->Type)
    {
      case resTemperature:
        isOurReservationType = (sensorType == StateTemperature);
      break;

      case resHumidity:
        isOurReservationType = (sensorType == StateHumidity);
      break;

      case resLuminosity:
        isOurReservationType = (sensorType == StateLuminosity);
      break;

      case resSoilMoisture:
        isOurReservationType = (sensorType == StateSoilMoisture);
      break;
      
    } // switch

    if(!isOurReservationType) // не наш тип резервирования, пропускаем
      continue;

    // теперь ищем - есть ли в списке резервирования датчик, с которого нет показаний
    bool isOurReservationList = false;

    for(size_t j=0;j<rec->Items.size();j++)
    {
      ReservationItem ri = rec->Items[j];
      if(ri.SensorIndex == sensorIndex)
      {
        // индексы совпадают, надо сравнить модули
        switch(ri.ModuleType)
        {
          case resModuleState:
            isOurReservationList = (sourceModule == moduleState);
          break;

          case resModuleHumidity:
            isOurReservationList = (sourceModule == moduleHumidity);
          break;

          case resModuleLuminosity:
            isOurReservationList = (sourceModule == moduleLuminosity);
          break;

          case resModuleSoilMoisture:
            isOurReservationList = (sourceModule == moduleSoilMoisture);
          break;
          
        } // switch

      } // if

        if(isOurReservationList) // наш лист, можем с ним работать
          break;

    } // for

    if(isOurReservationList)
    {
      // нашли наш список резервирования, теперь задача - получить из этого списка первое значение с данными

      // пробегаемся по всем записям списка резервации
      for(size_t k=0;k<rec->Items.size();k++)
      {
        ReservationItem ri = rec->Items[k];
        AbstractModule* workModule = NULL;

        // смотрим тип модуля, с которого резервируем показания
        switch(ri.ModuleType)
        {
          case resModuleState:
            workModule = moduleState;
          break;

          case resModuleHumidity:
            workModule = moduleHumidity;
          break;

          case resModuleLuminosity:
            workModule = moduleLuminosity;
          break;

          case resModuleSoilMoisture:
            workModule = moduleSoilMoisture;
          break;
          
        } // switch

        if(!workModule)
          continue; // не нашли модуль в прошивке, продолжаем искать

        // теперь надо получить с модуля показания нужного типа
        OneState* os = workModule->State.GetState(sensorType,ri.SensorIndex);
        if(os)
        {
          // есть такие показания, проверяем - если там есть данные - значит мы нашли резервирование
          if(os->HasData())
            return os; // возвращаем состояние
        }
        
      } // for
      
      
    } // if isOurReservationList
   
  } // for

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ReservationModule::Update(uint16_t dt)
{ 
  UNUSED(dt);
  // обновление модуля тут
  if(!bInited)
  {
    bInited = true;
    moduleState = MainController->GetModuleByID("STATE"); 
    moduleHumidity = MainController->GetModuleByID("HUMIDITY"); 
    moduleLuminosity = MainController->GetModuleByID("LIGHT"); 
    moduleSoilMoisture = MainController->GetModuleByID("SOIL");
    
    LoadReservations();

  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  ReservationModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer)
    PublishSingleton = UNKNOWN_COMMAND;

  uint8_t argsCount = command.GetArgsCount();

  if(command.GetType() == ctGET)
  {
    String which = command.GetArg(0);
    if(which == CNT_COMMAND)
    {
      PublishSingleton.Flags.Status = true;
      PublishSingleton = CNT_COMMAND;
      PublishSingleton << PARAM_DELIMITER;
      PublishSingleton << records.size();
      
    } // if(which == CNT_COMMAND)
    else
    if(which == VIEW_COMMAND)
    {
      // просмотр одного списка
      if(argsCount < 2)
      {
        PublishSingleton = PARAMS_MISSED;
      }
      else
      {
        uint8_t idx = atoi(command.GetArg(1));
        if(idx >= records.size())
        {
          PublishSingleton = PARAMS_MISSED;
        }
        else
        {
          PublishSingleton.Flags.Status = true;

          PublishSingleton = VIEW_COMMAND;
          PublishSingleton << PARAM_DELIMITER << idx << PARAM_DELIMITER;

          ReservationRecord* rec = records[idx];

          switch(rec->Type)
          {
            case resTemperature:
              PublishSingleton << F("TEMP");
            break;

            case resHumidity:
              PublishSingleton << "HUMIDITY";
            break;

            case resLuminosity:
              PublishSingleton << "LIGHT";
            break;

            case resSoilMoisture:
              PublishSingleton << "SOIL";
            break;
            
          } // switch

          for(size_t it = 0; it < rec->Items.size(); it++)
          {
            PublishSingleton << PARAM_DELIMITER;          
            ReservationItem resItem = rec->Items[it];
            switch(resItem.ModuleType)
            {
              case resModuleState:
                PublishSingleton << "STATE";
              break;

              case resModuleHumidity:
                PublishSingleton << "HUMIDITY";
              break;

              case resModuleLuminosity:
                PublishSingleton << "LIGHT";
              break;

              case resModuleSoilMoisture:
                PublishSingleton << "SOIL";
              break;

            } // switch

            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << resItem.SensorIndex;
          } // for
          
          
        } // else good index
        
      }
    } // VIEW_COMMAND
  }
  else
  if(command.GetType() == ctSET)
  {
     String which = command.GetArg(0);
     if(which == CC_DELETE_COMMAND) // CTSET=RSRV|DEL
     {
        PublishSingleton = REG_DEL;
        PublishSingleton.Flags.Status = true;
        ClearReservations();
      
     } // CC_DELETE_COMMAND
     else
     if(which == CC_SAVE_COMMAND)// CTSET=RSRV|SAVE
     {
        PublishSingleton = REG_SUCC;
        PublishSingleton.Flags.Status = true;
        SaveReservations();
   
     } // CC_SAVE_COMMAND
     else
     if(which == CC_ADD_COMMAND) // CTSET=RSRV|ADD|TYPE|Module1|Idx1|Module2|Idx2|ModuleN|IdxN
     {
        if(argsCount < 2 || argsCount % 2 != 0)
        {
          PublishSingleton = PARAMS_MISSED;
        }
        else
        {
          const char* type = command.GetArg(1);
          ReservationRecord* rec = new ReservationRecord;
          if(!strcmp_P(type,(const char*) F("TEMP")))
            rec->Type = resTemperature;
          else
            if(!strcmp_P(type,(const char*) "HUMIDITY"))
              rec->Type = resHumidity;
          else
            if(!strcmp_P(type,(const char*) "LIGHT"))
              rec->Type = resLuminosity;
          else
            if(!strcmp_P(type,(const char*) "SOIL"))
              rec->Type = resSoilMoisture;

         // теперь собираем параметры датчиков
           for(uint8_t i=2;i<argsCount;i+=2)
           {
              const char* moduleName = command.GetArg(i);
              uint8_t idx = atoi(command.GetArg(i+1));
  
              ReservationItem item; item.SensorIndex = idx;
              if(!strcmp_P(moduleName,(const char*) "STATE"))
                item.ModuleType = resModuleState;
              else  
              if(!strcmp_P(moduleName,(const char*) "HUMIDITY"))
                item.ModuleType = resModuleHumidity;
              else  
              if(!strcmp_P(moduleName,(const char*) "LIGHT"))
                item.ModuleType = resModuleLuminosity;
              else  
              if(!strcmp_P(moduleName,(const char*) "SOIL"))
                item.ModuleType = resModuleSoilMoisture;

              rec->Items.push_back(item);
  
            } // for

            records.push_back(rec);

            PublishSingleton.Flags.Status = true;
            PublishSingleton = REG_SUCC;
            
         } // else good args
          
     } // CC_ADD_COMMAND
     
  } // ctSET

  MainController->Publish(this,command);
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------

