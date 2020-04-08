#include "DynamicSensorsResetModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
DynamicSensorsResetModule* DynamicSensorsReset = NULL;
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
DynamicSensorsResetModule::DynamicSensorsResetModule() : AbstractModule("DRS") 
{
  #ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
    DynamicSensorsReset = this;
  #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DynamicSensorsResetModule::Setup()
{
  // настройка модуля тут
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DynamicSensorsResetModule::Update()
{ 
  // обновление модуля тут

  // проверяем - не отвалились ли динамически добавленные датчики?
  uint32_t now = millis();
  size_t sz = list.size();  
  for(size_t i=0;i<sz;i++)
  {
    if(now - list[i].LastDataAt > DYNAMIC_SENSORS_RESET_INTERVAL)
    {
      list[i].LastDataAt = now;
      list[i].State->Reset(); // сбрасываем показания, потому что их не было долгое время
    }
  } // for

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  DynamicSensorsResetModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  UNUSED(command);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
DynSensorReset* DynamicSensorsResetModule::findByState(OneState* state)
{
  size_t sz = list.size();
  for(size_t i=0;i<sz;i++)
  {
    if(list[i].State == state)
    {
      return &(list[i]);
    }
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DynamicSensorsResetModule::Observe(OneState* state)
{
  DynSensorReset* existing = findByState(state);
  if(existing)
  {
    existing->LastDataAt = millis();
    return;
  }

  DynSensorReset newData;
  newData.State = state;
  newData.LastDataAt = millis();
  list.push_back(newData);
}
//--------------------------------------------------------------------------------------------------------------------------------------

