#pragma once

#include "AbstractModule.h"
#include "HumidityGlobals.h"

#include "Si7021Support.h"
#include "DHTSupport.h"
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class HumidityModule : public AbstractModule // модуль управления влажностью
{
  private:

    HumidityAnswer dummyAnswer;
    HumidityAnswer QuerySensor(uint8_t sensorNumber, uint8_t pin, uint8_t pin2,HumiditySensorType type); // опрашивает сенсор
    Vector<uint8_t> si7021InitStatus;

    uint16_t lastUpdateCall;
    uint8_t lastSi7021StrobeBreakPin;

    
  public:
    HumidityModule() : AbstractModule("HUMIDITY")
    , lastUpdateCall(256) // разнесём опросы датчиков по времени
    {}

    bool ExecCommand(const Command& command,bool wantAnswer);
    void Setup();
     void Update();

};
//--------------------------------------------------------------------------------------------------------------------------------------

