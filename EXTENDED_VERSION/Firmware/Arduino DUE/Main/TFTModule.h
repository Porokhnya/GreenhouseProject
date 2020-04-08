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
    bool canUseMenu;
  
  public:
    TFTModule() : AbstractModule("TFT") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
      void halt();
    #endif

};
#endif // USE_TFT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
