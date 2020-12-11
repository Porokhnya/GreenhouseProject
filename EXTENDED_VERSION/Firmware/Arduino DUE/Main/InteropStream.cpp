#include "InteropStream.h"
#include "Globals.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------
InteropStream ModuleInterop;
//--------------------------------------------------------------------------------------------------------------------------------
InteropStream::InteropStream()
{
}
//--------------------------------------------------------------------------------------------------------------------------------
InteropStream::~InteropStream()
{
}
//--------------------------------------------------------------------------------------------------------------------------------
bool InteropStream::QueryCommand(COMMAND_TYPE cType, const String& command, bool isInternalCommand)
{
  String data = command; // копируем во внутренний буфер, т.к. входной параметр - const
   
  int delimIdx = data.indexOf('|');
  const char* params = NULL;
  if(delimIdx != -1)
  {
    data[delimIdx] = '\0';
    params = &(data[delimIdx+1]);
  }

  const char* moduleId = data.c_str();
  
  Command cmd;
  cmd.Construct(moduleId,params,cType);
 
  cmd.SetInternal(isInternalCommand); // устанавливаем флаг команды
  MainController->ProcessModuleCommand(cmd,NULL);
  return true;
    
}
//--------------------------------------------------------------------------------------------------------------------------------
BlinkModeInterop::BlinkModeInterop()
{

  blinkInterval = 0;
  timer = 0;
  pinState = LOW;
  canWorkWithPin = false;
  _timer = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
void BlinkModeInterop::update()
{
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;	
	
  if(!blinkInterval || !canWorkWithPin) // выключены
  {
    return;
  }

  timer += dt;

  if(timer < blinkInterval) // не настало время
  {
    return;
  }

  timer -= blinkInterval;

  DiodesBinding bnd = HardwareBinding->GetDiodesBinding();
  pinState = pinState == bnd.Level ? !bnd.Level : bnd.Level;
  out(pinState);
  
  /*
  pinState = pinState == LOW ? HIGH : LOW;
  
  #if INFO_DIODES_DRIVE_MODE == DRIVE_DIRECT
    WORK_STATUS.PinWrite(pin,pinState);
  #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23S17
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      WORK_STATUS.MCP_SPI_PinWrite(INFO_DIODES_MCP23S17_ADDRESS,pin,pinState);
    #endif
  #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23017
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      WORK_STATUS.MCP_I2C_PinWrite(INFO_DIODES_MCP23017_ADDRESS,pin,pinState);
    #endif
  #endif
  */  
}
//--------------------------------------------------------------------------------------------------------------------------------
void BlinkModeInterop::begin(uint8_t p)
{
  pin = p;
  canWorkWithPin = false;

  DiodesBinding bnd = HardwareBinding->GetDiodesBinding();
  if(bnd.LinkType != linkUnbinded)
  {
    pinState = !bnd.Level;
    
    if(bnd.LinkType == linkDirect)
    {
        if(pin != UNBINDED_PIN && pin > 1)
        {
          #ifndef DISABLE_DIODES_CONFIGURE
          if(EEPROMSettingsModule::SafePin(pin))
          #endif
          {
             WORK_STATUS.PinMode(pin,OUTPUT);
             WORK_STATUS.PinWrite(pin,!bnd.Level);             
             canWorkWithPin = true;
          }
        }       
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      if(pin != UNBINDED_PIN)
        {
             WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,pin,OUTPUT);
             WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,pin,!bnd.Level);
             canWorkWithPin = true;
        }

      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      if(pin != UNBINDED_PIN)
        {
             WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,pin,OUTPUT);
             WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,pin,!bnd.Level);
             canWorkWithPin = true;
        }
       
      #endif
    }    
    
  } // if(bnd.LinkType != linkUnbinded)

/*  
  #if INFO_DIODES_DRIVE_MODE == DRIVE_DIRECT
    WORK_STATUS.PinMode(pin,OUTPUT);
  #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23S17
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      WORK_STATUS.MCP_SPI_PinMode(INFO_DIODES_MCP23S17_ADDRESS,pin,OUTPUT);
    #endif
  #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23017
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      WORK_STATUS.MCP_I2C_PinMode(INFO_DIODES_MCP23017_ADDRESS,pin,OUTPUT);
    #endif
  #endif
*/  
}
//--------------------------------------------------------------------------------------------------------------------------------
void BlinkModeInterop::out(uint8_t level)
{
    if(!canWorkWithPin)
    {
      return;
    }

  DiodesBinding bnd = HardwareBinding->GetDiodesBinding();
  if(bnd.LinkType != linkUnbinded)
  {
    if(bnd.LinkType == linkDirect)
    {
        WORK_STATUS.PinWrite(pin,level);             
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
         WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,pin,level);
      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
         WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,pin,level);
      #endif
    }    
    
  } // if(bnd.LinkType != linkUnbinded)
    
}
//--------------------------------------------------------------------------------------------------------------------------------
void BlinkModeInterop::blink(uint16_t interval)
{
    if(!canWorkWithPin)
    {
      return;
    }  

  blinkInterval = interval;
  
  if(!blinkInterval) // просто выключить диод
  {
      DiodesBinding bnd = HardwareBinding->GetDiodesBinding();
      out(!bnd.Level);
    /*
    //WORK_STATUS.PinWrite(pin,LOW);
    #if INFO_DIODES_DRIVE_MODE == DRIVE_DIRECT
      WORK_STATUS.PinWrite(pin,LOW);
    #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23S17
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        WORK_STATUS.MCP_SPI_PinWrite(INFO_DIODES_MCP23S17_ADDRESS,pin,LOW);
      #endif
    #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23017
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        WORK_STATUS.MCP_I2C_PinWrite(INFO_DIODES_MCP23017_ADDRESS,pin,LOW);
      #endif
    #endif
    */      
  }

}
//--------------------------------------------------------------------------------------------------------------------------------


