#ifndef _TFT_MODULE_H
#define _TFT_MODULE_H

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TFT_MODULE
#include "UTFTMenu.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class TFTModule : public AbstractModule // модуль поддержки 7'' TFT
{
  private:

    TFTMenu myTFTMenu;
  
  public:
    TFTModule() : AbstractModule("TFT") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
#endif // USE_TFT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
