#ifndef _PINMODULE_H
#define _PINMODULE_H

#include "Globals.h"
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t pinNumber;
  byte flags; // первый бит - isActive, второй - hasChanges, третий - pinState
  
} PIN_STATE;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<PIN_STATE> PIN_STATES;
//--------------------------------------------------------------------------------------------------------------------------------------
class PinModule : public AbstractModule
{
  private:

    PIN_STATES pinStates;
    void UpdatePinStates();

    uint8_t GetPinState(uint8_t pinNumber);
    PIN_STATE* AddPin(uint8_t pinNumber,uint8_t currentState);
    bool PinExist(uint8_t pinNumber);
    PIN_STATE* GetPin(uint8_t pinNumber);
   
  public:
    PinModule() : AbstractModule("PIN") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
