#include "InteropStream.h"
#include "Globals.h"
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
}
//--------------------------------------------------------------------------------------------------------------------------------
void BlinkModeInterop::update(uint16_t dt)
{
  if(!blinkInterval) // выключены
    return;

  timer += dt;

  if(timer < blinkInterval) // не настало время
    return;

  timer -= blinkInterval;
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
}
//--------------------------------------------------------------------------------------------------------------------------------
void BlinkModeInterop::begin(uint8_t p)
{
  pin = p;
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
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void BlinkModeInterop::blink(uint16_t interval)
{

  blinkInterval = interval;
  
  if(!blinkInterval)
  {
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
  }

}
//--------------------------------------------------------------------------------------------------------------------------------


