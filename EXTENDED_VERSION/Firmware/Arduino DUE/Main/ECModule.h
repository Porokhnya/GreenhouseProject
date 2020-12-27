#pragma once

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class ECModule : public AbstractModule // модуль контроля EC
{
  private:
  public:
    ECModule() : AbstractModule("EC") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

};
//--------------------------------------------------------------------------------------------------------------------------------------

