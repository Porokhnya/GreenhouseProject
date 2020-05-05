#ifndef _DELTA_MODULE_H
#define _DELTA_MODULE_H

#include "AbstractModule.h"
#include "Settings.h"
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t SensorType; // тип сенсора
  AbstractModule* Module1; // первый модуль, с которого мы запрашиваем показания
  AbstractModule* Module2; // второй модуль, с которого мы запрашиваем показания
  uint8_t SensorIndex1; // индекс сенсора в первом модуле
  uint8_t SensorIndex2; // индекс сенсора во втором модуле
  
} DeltaSettings; // настройки одной дельты
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<DeltaSettings> DeltasVector; // наш вектор с дельтами
//--------------------------------------------------------------------------------------------------------------------------------------
class DeltaModule; // forward declaration
//--------------------------------------------------------------------------------------------------------------------------------------
class DeltaModule : public AbstractModule // модуль регистрации дельт с показаний датчиков
{
  private:

//  GlobalSettings* settings; // указатель на настройки
  bool isDeltasInited; // флаг, что мы инициализировали настройки дельт
  uint16_t lastUpdateCall;

  DeltasVector deltas; // наши дельты будут здесь
  size_t deltaReadIndex; // текущий индекс чтения дельты (для сохранения настроек)
  
  static void OnDeltaSetCount(uint8_t& count); // нам передали кол-во сохранённых в EEPROM дельт
  static void OnDeltaRead(uint8_t& sensorType, String& moduleName1,uint8_t& sensorIdx1, String& moduleName2, uint8_t& sensorIdx2); // нам передали прочитанные из EEPROM данные одной дельты
  static void OnDeltaGetCount(uint8_t& count); // у нас запросили - сколько установок дельт писать в EEPROM
  static void OnDeltaWrite(uint8_t& sensorType, String& moduleName1,uint8_t& sensorIdx1, String& moduleName2, uint8_t& sensorIdx2); // мы передаём данные очередной дельты

  static DeltaModule* _thisDeltaModule;

  void InitDeltas();
  void UpdateDeltas();
  void SaveDeltas();
  
  public:
    DeltaModule() : AbstractModule("DELTA"), lastUpdateCall(876) {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
