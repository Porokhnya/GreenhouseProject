#ifndef _HUMIDITY_MODULE_H
#define _HUMIDITY_MODULE_H

#include "AbstractModule.h"
#include "HumidityGlobals.h"

#include "Si7021Support.h"
#include "DHTSupport.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t pin;
  uint8_t pin2;
  HumiditySensorType type;
  
} HumiditySensorRecord;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
class HumidityModule : public AbstractModule // модуль управления влажностью
{
  private:

#if SUPPORTED_HUMIDITY_SENSORS > 0
    //DHTSupport dhtQuery; // класс опроса датчиков DHT
   // Si7021 si7021; // класс опроса датчиков Si7021
    HumidityAnswer dummyAnswer;
    HumidityAnswer QuerySensor(uint8_t sensorNumber, uint8_t pin, uint8_t pin2,HumiditySensorType type); // опрашивает сенсор
#endif

    uint16_t lastUpdateCall;

    uint8_t lastSi7021StrobeBreakPin;

    
  public:
    HumidityModule() : AbstractModule("HUMIDITY")
    , lastUpdateCall(256) // разнесём опросы датчиков по времени
    {}

    bool ExecCommand(const Command& command,bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
