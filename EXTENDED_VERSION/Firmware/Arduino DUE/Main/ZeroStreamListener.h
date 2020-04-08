#ifndef _ZERO_STREAM_LISTENER_H
#define _ZERO_STREAM_LISTENER_H

#include "AbstractModule.h"
#include "Globals.h"
#include "UniversalSensors.h"
//--------------------------------------------------------------------------------------------------------------------------------------
// класс модуля "0"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA_GATE
  extern UniLoRaGate loraGate;
#endif
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
    void Update();

    #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
      String getSKpart()
      {
        return PROTECT_KEY2;
      }
    #endif    
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern ZeroStreamListener* ZeroStream;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
