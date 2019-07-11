#include "DeltaModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
DeltaModule* DeltaModule::_thisDeltaModule = NULL; // указатель на экземпляр класса
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::OnDeltaSetCount(uint8_t& count)
{ 
  // нам передали кол-во сохранённых в EEPROM дельт
  UNUSED(count);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::OnDeltaRead(uint8_t& _sensorType, String& moduleName1,uint8_t& sensorIdx1, String& moduleName2, uint8_t& sensorIdx2)
{
  // нам передали прочитанные из EEPROM данные одной дельты
  // вызываем yield, поскольку чтение из EEPROM занимает время.
  yield();

  ModuleStates sensorType = (ModuleStates) _sensorType; // приводим тип

  // сначала получаем два модуля, они должны быть уже зарегистрированы, поскольку мы инициализируем дельты в методе Update, который вызывается уже в loop().
  DeltaSettings ds;
  ds.Module1 = MainController->GetModuleByID(moduleName1);
  ds.Module2 = MainController->GetModuleByID(moduleName2);

  // проверяем, всё ли мы получили правильно
  if(!(ds.Module1 && ds.Module2))
  {
    return;
  }

  // теперь проверяем, не указывает ли какое-либо имя модуля на нас.
  if(ds.Module1 == DeltaModule::_thisDeltaModule || ds.Module2 == DeltaModule::_thisDeltaModule)
  {
    return;
  }

  //теперь проверяем, есть ли у обеих модулей датчики указанного типа
 if(!(ds.Module1->State.HasState(sensorType) && ds.Module2->State.HasState(sensorType)))
 {
  return;
 }

  //теперь проверяем, правильные ли индексы датчиков переданы
 if(!(ds.Module1->State.GetState(sensorType,sensorIdx1) && ds.Module2->State.GetState(sensorType,sensorIdx2)))
 {
  return;
 } 

 // если мы здесь, значит - оба модуля есть в системе, ни один из модулей не указывает на нас, и у каждого из модулей есть датчики определённого типа,
 // плюс индексы назначены правильно.
 // следовательно - мы можем сохранять структуру в векторе.
 ds.SensorType = sensorType;
 ds.SensorIndex1 = sensorIdx1;
 ds.SensorIndex2 = sensorIdx2;

 // теперь не забываем добавить своё внутреннее состояние, которое будет дёргать модуль ALERT, получая показания
 DeltaModule::_thisDeltaModule->State.AddState(sensorType,DeltaModule::_thisDeltaModule->deltas.size()); // индексом виртуального датчика будет размер массива, т.е. автоматически увеличиваться с каждой новой настройкой.

 // записывать в состояние пока ничего не надо, поскольку State сам инициализирует всё значениями по умолчанию.

 // сохраняем настройки в структуру
 DeltaModule::_thisDeltaModule->deltas.push_back(ds);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::OnDeltaGetCount(uint8_t& count)
{
  // у нас запросили - сколько установок дельт писать в EEPROM
  count = (uint8_t) DeltaModule::_thisDeltaModule->deltas.size();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::OnDeltaWrite(uint8_t& sensorType, String& moduleName1,uint8_t& sensorIdx1, String& moduleName2, uint8_t& sensorIdx2)
{
  // мы передаём данные очередной дельты
  // вызываем yield, поскольку запись в EEPROM занимает время.
  yield();

  // получили указатель на структуру
  DeltaSettings* ds = &(DeltaModule::_thisDeltaModule->deltas[DeltaModule::_thisDeltaModule->deltaReadIndex]);
  // передаём её значения
  sensorType = ds->SensorType;
  moduleName1 = ds->Module1->GetID();
  sensorIdx1 = ds->SensorIndex1;
  moduleName2 = ds->Module2->GetID();
  sensorIdx2 = ds->SensorIndex2;

  // передали, увеличили указатель чтения
  DeltaModule::_thisDeltaModule->deltaReadIndex++;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::Setup()
{
  // настройка модуля тут
  isDeltasInited = false;
  //settings = MainController->GetSettings();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::SaveDeltas()
{
  // сохраняем дельты в EEPROM
  deltaReadIndex = 0;
  
  DeltaModule::_thisDeltaModule = this; // сохраняем указатель на себя

  // читаем данные из EEPROM
  MainController->GetSettings()->WriteDeltaSettings(OnDeltaGetCount, OnDeltaWrite);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::Update(uint16_t dt)
{ 

  // обновление модуля тут
  if(!isDeltasInited)
  {
    // инициализируем дельты здесь, поскольку при вызове Setup настройки уже загружены, но наш модуль ещё не зарегистрирован в контроллере
    isDeltasInited = true;
    InitDeltas();
  }

  lastUpdateCall += dt;
  if(lastUpdateCall < DELTA_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
    return;
  else
    lastUpdateCall = 0;
  
  UpdateDeltas(); // обновляем дельты

}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::UpdateDeltas()
{
  // обновляем дельты тут. Проходим по всем элементам массива, смотрим, чего там лежит, получаем показания с нужных датчиков - и сохраняем дельты у себя.
  size_t cnt = deltas.size();
  for(size_t i=0;i<cnt;i++)
  {
    DeltaSettings* ds = &(deltas[i]);
    // получили первую настройку дельты, работаем с ней

    // получаем значения двух датчиков

    // первого...
    OneState* os1 = ds->Module1->State.GetState((ModuleStates)ds->SensorType,ds->SensorIndex1);

    // и второго
    OneState* os2 = ds->Module2->State.GetState((ModuleStates)ds->SensorType,ds->SensorIndex2);

    OneState* deltaState = State.GetState((ModuleStates)ds->SensorType,i); // получаем наше состояние

    // и сохраняем в него дельту, индекс при этом должен остаться нетронутым
    if(deltaState && os1 && os2)
      *deltaState = (*os1 - *os2);

  } // for

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DeltaModule::InitDeltas()
{
  // загружаем дельты из EEPROM
  deltas.clear();

  DeltaModule::_thisDeltaModule = this; // сохраняем указатель на себя

  // читаем данные из EEPROM
  MainController->GetSettings()->ReadDeltaSettings(OnDeltaSetCount, OnDeltaRead);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  DeltaModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer)
    PublishSingleton = UNKNOWN_COMMAND;

  size_t argsCount = command.GetArgsCount();
    
  if(command.GetType() == ctGET)
  {
    if(!argsCount)
    {
      if(wantAnswer)
        PublishSingleton = PARAMS_MISSED;
      
    } // !argsCount
    else
    {
      String arg = command.GetArg(0);
      if(arg == DELTA_COUNT_COMMAND) // получить кол-во дельт, CTGET=DELTA|CNT
      {
        if(wantAnswer)
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = DELTA_COUNT_COMMAND;
          PublishSingleton << PARAM_DELIMITER << deltas.size();
        }
        
      } // DELTA_COUNT_COMMAND
      else
      if(arg == DELTA_VIEW_COMMAND) // просмотр дельты по индексу, CTGET=DELTA|VIEW|0
      {
         if(argsCount < 2)
         {
          if(wantAnswer)
            PublishSingleton = PARAMS_MISSED;
         } // argsCount < 2
         else
         {
           //arg = command.GetArg(1);
           size_t deltaIdx = atoi(command.GetArg(1));
           if(deltaIdx >= deltas.size())
           {
            if(wantAnswer)
              PublishSingleton = PARAMS_MISSED;
              
           } // bad index
           else
           {
            if(wantAnswer)
            {
              PublishSingleton.Flags.Status = true;
              PublishSingleton = DELTA_VIEW_COMMAND;
              PublishSingleton << PARAM_DELIMITER << deltaIdx << PARAM_DELIMITER;

              DeltaSettings* ds = &(deltas[deltaIdx]);
              
              String tp = OneState::GetStringType((ModuleStates)ds->SensorType); // тип датчика

              PublishSingleton << tp << PARAM_DELIMITER << (ds->Module1->GetID()) << PARAM_DELIMITER << ds->SensorIndex1
              << PARAM_DELIMITER << (ds->Module2->GetID()) << PARAM_DELIMITER << ds->SensorIndex2;
              
            } // wantAnswer
           } // else good index
         } // else
      } // DELTA_VIEW_COMMAND
      
    } // have args
    
  } // GET
  else
  if(command.GetType() == ctSET)
  {
    if(!argsCount)
    {
      if(wantAnswer)
        PublishSingleton = PARAMS_MISSED;
    } // !argsCount
    else
    {
       String arg = command.GetArg(0);
       
       if(arg == DELTA_SAVE_COMMAND) // сохранить все настройки дельт в EEPROM, CTSET=DELTA|SAVE
       {
          if(wantAnswer)
          {
            PublishSingleton.Flags.Status = true;
            PublishSingleton = DELTA_SAVE_COMMAND;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }

          SaveDeltas(); // сохраняем дельты
       } // DELTA_SAVE_COMMAND
       else
       if(arg == DELTA_DELETE_COMMAND) // удалить все дельты, CTSET=DELTA|DEL
       {
          if(wantAnswer)
          {
            PublishSingleton.Flags.Status = true;
            PublishSingleton = DELTA_DELETE_COMMAND;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }

          // убираем свои состояния, чтобы модуль ALERT не дёргал там, где ничего нет
          size_t cnt = deltas.size();
          for(size_t i=0;i<cnt;i++)
          {
            DeltaSettings* ds = &(deltas[i]);
            State.RemoveState((ModuleStates)ds->SensorType,i); // просим класс состояний удалить состояние
          } // for

          deltas.clear(); // чистим дельты
          SaveDeltas(); // сохраняем дельты
        
       } // DELTA_DELETE_COMMAND
       else
       if(arg = DELTA_ADD_COMMAND) // добавить дельту, CTSET=DELTA|ADD|SensorType|ModuleName1|SensorIndex1|ModuleName2|SensorIndex2
       {
          if(argsCount < 6)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          } // argsCount < 6
          else
          {
            // парсим аргументы
            DeltaSettings ds; // сюда будем сохранять
            uint8_t readIdx = 1;

            ds.SensorType = OneState::GetType(command.GetArg(readIdx++));

            String moduleName1 = command.GetArg(readIdx++); // читаем имя первого модуля
            ds.SensorIndex1 = (uint8_t) atoi(command.GetArg(readIdx++));
            
            String moduleName2 = command.GetArg(readIdx++); // читаем имя второго модуля
            ds.SensorIndex2 = (uint8_t) atoi(command.GetArg(readIdx++));

            ds.Module1 = MainController->GetModuleByID(moduleName1);
            ds.Module2 = MainController->GetModuleByID(moduleName2);

            // проверяем все параметры
            if(!ds.SensorType || // если тип датчика не задан
            !(ds.Module1 && ds.Module2) || // или один из модулей не найден
            ds.Module1 == this || ds.Module2 == this || // или любой из них ссылается на нас
            !ds.Module1->State.HasState((ModuleStates)ds.SensorType) || // или у первого нет нужного типа датчика
            !ds.Module2->State.HasState((ModuleStates)ds.SensorType) || // или у второго нет нужного типа датчика
            !ds.Module1->State.GetState((ModuleStates)ds.SensorType,ds.SensorIndex1) || // или переданный индекс первого датчика неправильный
            !ds.Module2->State.GetState((ModuleStates)ds.SensorType,ds.SensorIndex2) // или переданный индекс второго датчика неправильный
            )
            {
              // чего-то пошло не так
              if(wantAnswer)
                PublishSingleton = PARAMS_MISSED;
            }
            else // иначе - всё зашибись, и мы можем добавлять дельту
            {
              if(deltas.size() >= MAX_DELTAS) // превышен лимит дельт
              {
              if(wantAnswer)
                PublishSingleton = UNKNOWN_COMMAND;
              }
              else
              {
                  // можем добавлять дельту, сохранением занимается команда SAVE.
                  // добавляем своё состояние
                  State.AddState((ModuleStates)ds.SensorType,deltas.size());
                  // теперь сохраняем структуру в вектор.
                  deltas.push_back(ds);
                  
                  if(wantAnswer)
                  {
                    PublishSingleton.Flags.Status = true;
                    PublishSingleton = DELTA_ADD_COMMAND;
                    PublishSingleton << PARAM_DELIMITER << REG_SUCC << PARAM_DELIMITER << (deltas.size() - 1);
                  } // wantAnswer
              } // else can add
              
            } // good params
            
          } // enough args
       } // DELTA_ADD_COMMAND
    } // have args
      
  } // SET


  MainController->Publish(this,command);
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
