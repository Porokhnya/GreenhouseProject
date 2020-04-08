#include "TFTModule.h"
#include "ModuleController.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TFT_MODULE
void TFTModule::Setup()
{
  canUseMenu = false;
  TFTBinding bnd = HardwareBinding->GetTFTBinding();

  if(bnd.DisplayType == 1 || bnd.DisplayType == 2)
  {
    canUseMenu = true;
  }
  
  if(canUseMenu)
  {
    myTFTMenu.setup();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TFTModule::Update()
{ 
  // обновление модуля тут

  if(canUseMenu)
  {
    myTFTMenu.update();
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  TFTModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  UNUSED(command);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
//--------------------------------------------------------------------------------------------------------------------------------------
void TFTModule::halt()
{
  if(canUseMenu)
  {
    myTFTMenu.switchToScreen("UNRK");
  }
  
  Update();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_TFT_MODULE

