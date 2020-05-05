#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SOIL_MOISTURE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
enum
{
  SOIL_WAIT_INTERVAL,
  SOIL_WAIT_POWER
};
//--------------------------------------------------------------------------------------------------------------------------------------
class SoilMoistureModule : public AbstractModule // модуль датчиков влажности почвы
{
  private:
  
    uint16_t lastUpdateCall;
    uint8_t machineState;

    void readFromSensors();
  
  public:
    SoilMoistureModule() : AbstractModule("SOIL"), lastUpdateCall(SOIL_MOISTURE_UPDATE_INTERVAL-387) {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
#endif // USE_SOIL_MOISTURE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------

