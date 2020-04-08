#pragma once

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class ModuleFish : public AbstractModule // заготовка для модуля
{
  private:
  public:
    ModuleFish(const char* id) : AbstractModule(id) {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

};
//--------------------------------------------------------------------------------------------------------------------------------------

