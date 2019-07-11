#ifndef _ZERO_STREAM_LISTENER_H
#define _ZERO_STREAM_LISTENER_H

#include "AbstractModule.h"
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
// класс модуля "0"
//--------------------------------------------------------------------------------------------------------------------------------------
class ZeroStreamListener : public AbstractModule
{
  private:
    void PrintSensorsValues(uint8_t totalCount,ModuleStates wantedState,AbstractModule* module, Stream* outStream);
    String GetGUID(const char* passedGuid);
  public:
    ZeroStreamListener() : AbstractModule("0") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
