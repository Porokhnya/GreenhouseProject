#ifndef _STAT_MODULE_H
#define _STAT_MODULE_H

#include "AbstractModule.h"
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
int freeRam();
//--------------------------------------------------------------------------------------------------------------------------------------
class StatModule : public AbstractModule
{
  private:
   unsigned long uptime;
  public:
    StatModule() : AbstractModule("STAT") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
