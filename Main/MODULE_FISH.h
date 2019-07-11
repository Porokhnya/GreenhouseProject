#ifndef _MODULE_FISH_H
#define _MODULE_FISH_H

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class ModuleFish : public AbstractModule // заготовка для модуля
{
  private:
  public:
    ModuleFish(const char* id) : AbstractModule(id) {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
