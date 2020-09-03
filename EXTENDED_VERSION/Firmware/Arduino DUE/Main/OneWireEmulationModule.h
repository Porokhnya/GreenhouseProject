#pragma once

#include "AbstractModule.h"
#include "DS18B20Query.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class OneWireEmulationModule : public AbstractModule // заготовка для модуля
{
  private:

    DS18B20Dispatcher lineManager;
  
  public:
    OneWireEmulationModule() : AbstractModule("1WEMUL") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

};
//--------------------------------------------------------------------------------------------------------------------------------------

