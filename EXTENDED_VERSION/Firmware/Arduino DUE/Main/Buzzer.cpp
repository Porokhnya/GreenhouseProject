#include "DelayedEvents.h"
#include "Buzzer.h"
#include "AbstractModule.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_BUZZER
//--------------------------------------------------------------------------------------------------------------------------------------
BuzzerClass Buzzer;
//--------------------------------------------------------------------------------------------------------------------------------------
BuzzerClass::BuzzerClass()
{
  active = false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::begin()
{

  BuzzerBinding bnd = HardwareBinding->GetBuzzerBinding();

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN && bnd.Pin > 1)
    {
      #ifndef DISABLE_BUZZER_CONFIGURE
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      #endif
      {
        WORK_STATUS.PinMode(bnd.Pin,OUTPUT);
        WORK_STATUS.PinWrite(bnd.Pin,!bnd.Level);        
      }
    }
  }
  else
  if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);      
    }
    #endif
  }
  else
  if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);      
    }
    #endif
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::stop()
{
  buzzLevel(false);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::buzz()
{
  if(active)
    return;

  active = true;
  buzzLevel(true);

  BuzzerBinding bnd = HardwareBinding->GetBuzzerBinding();
  
  CoreDelayedEvent.raise(bnd.Duration,buzzOffHandler,this);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::buzzLevel(bool on)
{

  BuzzerBinding bnd = HardwareBinding->GetBuzzerBinding();

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN && bnd.Pin > 1)
    {
      #ifndef DISABLE_BUZZER_CONFIGURE
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      #endif
      {
        WORK_STATUS.PinWrite(bnd.Pin,on ? bnd.Level : !bnd.Level);        
      }
    }
  }
  else
  if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,on ? bnd.Level : !bnd.Level);      
    }
    #endif
  }
  else
  if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,on ? bnd.Level : !bnd.Level);      
    }
    #endif
  }      
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::buzzOffHandler(void* param)
{
  BuzzerClass* bc = (BuzzerClass*) param;
  bc->buzzLevel(false);
  bc->active = false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_BUZZER
