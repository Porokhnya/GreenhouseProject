#ifndef _RESERVATION_MODULE_H
#define _RESERVATION_MODULE_H

#include "TinyVector.h"
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  resTemperature, // резервируем температуру
  resHumidity, // резервируем влажность
  resLuminosity, // резервируем освещённость
  resSoilMoisture // резервируем влажность почвы
  
} ReservationType; // тип резервирования
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  resModuleState, // STATE
  resModuleHumidity, // HUMIDITY
  resModuleLuminosity, // LIGHT
  resModuleSoilMoisture // SOIL
  
} ReservationModuleType; // модуль, с которого резервируем
//--------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t ModuleType : 4; // максимум 16 модулей
  uint8_t SensorIndex : 4; // максимум 16 датчиков
  
} ReservationItem; // одна запись о резервировании
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<ReservationItem> ReservationItems; // список резервирования
//--------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t Type; // тип резервирования (температура, влажность и т.п.)
  ReservationItems Items;
  
} ReservationRecord; // одна запись о резервировании
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<ReservationRecord*> ReservationRecords; // список резервирования
//--------------------------------------------------------------------------------------------------------------------------------
class ReservationModule : public AbstractModule, public ReservationResolver // модуль резервирования датчиков
{
  private:

  AbstractModule *moduleState, *moduleHumidity, *moduleLuminosity, *moduleSoilMoisture;
  ReservationRecords records;

  bool bInited;
  void LoadReservations();
  void ClearReservations();
  void SaveReservations();
  
  public:
    ReservationModule() : AbstractModule("RSRV") {bInited = false; moduleState = NULL; moduleHumidity = NULL; moduleLuminosity = NULL; moduleSoilMoisture = NULL;}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

    OneState* GetReservedState(AbstractModule* sourceModule, ModuleStates sensorType, uint8_t sensorIndex);

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
