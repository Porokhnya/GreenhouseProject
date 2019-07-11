#include "DelayedEvents.h"
#include "Buzzer.h"
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_BUZZER
//--------------------------------------------------------------------------------------------------------------------------------------
BuzzerClass Buzzer;
//--------------------------------------------------------------------------------------------------------------------------------------
BuzzerClass::BuzzerClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::begin()
{
      
      #if BUZZER_DRIVE_MODE == DRIVE_DIRECT
      
        WORK_STATUS.PinMode(BUZZER_DRIVE_PIN,OUTPUT);
        WORK_STATUS.PinWrite(BUZZER_DRIVE_PIN,BUZZER_OFF);
        
      #elif BUZZER_DRIVE_MODE == DRIVE_MCP23S17
      
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        
          WORK_STATUS.MCP_SPI_PinMode(BUZZER_MCP23S17_ADDRESS,BUZZER_DRIVE_PIN,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(BUZZER_MCP23S17_ADDRESS,BUZZER_DRIVE_PIN,BUZZER_OFF);
          
        #endif
        
      #elif BUZZER_DRIVE_MODE == DRIVE_MCP23017
      
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        
          WORK_STATUS.MCP_I2C_PinMode(BUZZER_MCP23017_ADDRESS,BUZZER_DRIVE_PIN,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(BUZZER_MCP23017_ADDRESS,BUZZER_DRIVE_PIN,BUZZER_OFF);
          
        #endif
        
      #endif

}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::buzz()
{
  buzzLevel(BUZZER_ON);
  CoreDelayedEvent.raise(BUZZER_DURATION,buzzOffHandler,this);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::buzzLevel(uint8_t level)
{
  #if BUZZER_DRIVE_MODE == DRIVE_DIRECT
      
        WORK_STATUS.PinWrite(BUZZER_DRIVE_PIN,level);
        
      #elif BUZZER_DRIVE_MODE == DRIVE_MCP23S17
      
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        
          WORK_STATUS.MCP_SPI_PinWrite(BUZZER_MCP23S17_ADDRESS,BUZZER_DRIVE_PIN,level);
          
        #endif
        
      #elif BUZZER_DRIVE_MODE == DRIVE_MCP23017
      
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        
          WORK_STATUS.MCP_I2C_PinWrite(BUZZER_MCP23017_ADDRESS,BUZZER_DRIVE_PIN,level);
          
        #endif
        
      #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BuzzerClass::buzzOffHandler(void* param)
{
  BuzzerClass* bc = (BuzzerClass*) param;
  bc->buzzLevel(BUZZER_OFF);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_BUZZER
