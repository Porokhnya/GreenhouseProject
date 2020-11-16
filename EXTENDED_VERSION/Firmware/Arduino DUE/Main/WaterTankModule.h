#pragma once

#include "AbstractModule.h"
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  waterTankNoErrors = 0, // нет ошибок
  waterTankNoData = 1, // нет внешних данных в течение долгого времени
  waterTankFullSensorError = 2, // не сработал датчик верхнего уровня в процессе наполнения, по превышению максимального времени наполнения
  waterTankBottomSensorFailure = 3, // ошибка нижнего датчика критического уровня
  
} WaterTankErrorType;
//--------------------------------------------------------------------------------------------------------------------------------------
class WaterTankModule : public AbstractModule // модуль управления уровнем воды в баке
{
  private:
    bool valveOnFlag;
    uint8_t fillStatus;
    bool errorFlag;
    uint8_t errorType;

    uint32_t lastDataPacketSeenAt;
    bool isOnline;
    
  public:
    WaterTankModule() : AbstractModule("WTANK") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();


    void FillTank(bool on); // команда по наполнению бака, параметр on - включить или выключить
    void UpdateState(bool isValveOn,uint8_t fillStatus,bool errorFlag,uint8_t errorType);

    uint8_t GetFillStatus() { return fillStatus; }
    bool HasErrors() { return errorFlag; }
    uint8_t GetErrorType() { return errorType; }
    bool IsValveOn() { return valveOnFlag; }
    String GetErrorText();

    bool IsModuleOnline();

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern WaterTankModule* WaterTank;
