#ifndef _WATERFLOW_MODULE_H
#define _WATERFLOW_MODULE_H

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
struct WaterflowStruct
{
unsigned long flowMilliLitres; // сколько миллилитров вылито с момента последнего замера
unsigned long totalMilliliters; // сюда накапливаем, пока не наберётся литр
unsigned long totalLitres; // сколько всего литров вылито через датчик
uint8_t calibrationFactor; // фактор калибровки
};
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
class WaterflowModule : public AbstractModule // модуль учёта расхода воды
{
  private:

  WaterflowStruct pin2Flow; // читаем на пине 2
  WaterflowStruct pin3Flow; // читаем на пине 3
  unsigned int checkTimer; // таймер для обновления данных

  void UpdateFlow(WaterflowStruct* wf,unsigned int delta, unsigned int pulses, uint8_t writeOffset);
  
  public:
    WaterflowModule() : AbstractModule("FLOW") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
