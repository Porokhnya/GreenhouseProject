#pragma once

#include "AbstractModule.h"
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  OneState* State;
  uint32_t LastDataAt;
  
} DynSensorReset;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<DynSensorReset> DynSensorResetList;
//--------------------------------------------------------------------------------------------------------------------------------------
class DynamicSensorsResetModule : public AbstractModule
{
  private:
      DynSensorResetList list;      
      DynSensorReset* findByState(OneState* state);
  public:
    DynamicSensorsResetModule();

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    void Observe(OneState* state); // наблюдаем за датчиком

};
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
  extern DynamicSensorsResetModule* DynamicSensorsReset;
#endif
//--------------------------------------------------------------------------------------------------------------------------------------

