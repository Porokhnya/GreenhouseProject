#include "PowerManager.h"
#include "ModuleController.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
PowerManagerClass PowerManager;
//--------------------------------------------------------------------------------------------------------------------------------------
PowerManagerClass::PowerManagerClass()
{
  bOnTimer = false;
  bOffTimer = false;
  offTimer = 0;
  bOn = true;
  timer = delayCounter = 0;
  windowsState = 0;
  pump1On = false;
  pump2On = false;
  wateringState = 0;
  doorsState = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::begin()
{
  #ifdef POWER_DEBUG
  DEBUG_LOGLN(F("PWR: begin..."));
  #endif
  
  WPowerBinding bnd = HardwareBinding->GetWPowerBinding();

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    #ifdef POWER_DEBUG
    DEBUG_LOGLN(F("PWR: NO BINDING!"));
    #endif
    return;
  }

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.Pin))
    {
      #ifdef POWER_DEBUG
      DEBUG_LOGLN(F("PWR: direct pin."));
      #endif
      WORK_STATUS.PinMode(bnd.Pin,OUTPUT);
    }
  }
  else
  if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    #ifdef POWER_DEBUG
      DEBUG_LOGLN(F("PWR: MCP23S17 pin."));
    #endif
      WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
    #endif
  }
  else
  if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    #ifdef POWER_DEBUG
      DEBUG_LOGLN(F("PWR: MCP23017 pin."));
    #endif
      WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
    #endif
  }

  out(false);

#ifdef POWER_DEBUG
  DEBUG_LOGLN(F("PWR: begin DONE."));
#endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::out(bool state)
{
#ifdef POWER_DEBUG  
  DEBUG_LOG(F("PWR: "));
  DEBUG_LOGLN(state ? F("ON") : F("OFF"));
#endif  
  
  if(bOn == state)
  {
#ifdef POWER_DEBUG    
    DEBUG_LOGLN(F("PWR: state unchanged!"));
#endif    
    return;
  }

  bOn = state;
  
   WPowerBinding bnd = HardwareBinding->GetWPowerBinding();

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
#ifdef POWER_DEBUG    
    DEBUG_LOGLN(F("PWR: NO BINDING!"));
#endif    
    return;
  }

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.Pin))
    {
#ifdef POWER_DEBUG      
      DEBUG_LOG(F("PWR: write to pin "));
      DEBUG_LOGLN(String(bnd.Pin));
#endif      
      WORK_STATUS.PinWrite(bnd.Pin,bOn ? bnd.Level : !bnd.Level);
    }
  }
  else
  if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
#ifdef POWER_DEBUG    
      DEBUG_LOG(F("PWR: write to MCP #"));
      DEBUG_LOG(String(bnd.MCPAddress));
      DEBUG_LOG(F(" and channel #"));
      DEBUG_LOGLN(String(bnd.Pin));
#endif      
      WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,bOn ? bnd.Level : !bnd.Level);
    #endif
  }
  else
  if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
#ifdef POWER_DEBUG    
      DEBUG_LOG(F("PWR: write to MCP #"));
      DEBUG_LOG(String(bnd.MCPAddress));
      DEBUG_LOG(F(" and channel #"));
      DEBUG_LOGLN(String(bnd.Pin));
#endif      
      WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,bOn ? bnd.Level : !bnd.Level);
    #endif
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::update()
{
  // проверяем на задержку выключения
  if(bOffTimer)
  {
    WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
    if(bnd.LinkType != linkUnbinded)
    {
      if(millis() - offTimer >= bnd.PowerOffDelay)
      {        
        if(isNoConsumers()) // потребителей нет, можно выключать
        {
#ifdef POWER_DEBUG          
          DEBUG_LOGLN(F("PWR: OFF timer done, no consumers, switch to OFF!"));
#endif          
          bOnTimer = false; // сбрасываем таймер включения, т.к. никто никуда не движется, выход будет выключен, следовательно - надо перед первым включением вернуть полную задержку
          bOffTimer = false;
          out(false);
        }
        else
        {
          offTimer = millis(); // есть ещё какие-то потребители, проверим позже
        }
      }
    }
  }
  
  if(!bOnTimer)
  {
    return;
  }

  uint32_t now = millis();
  uint16_t diff = now - timer;
  timer = now;

  if(delayCounter >= diff)
  {
    delayCounter -= diff;
  }
  else
  {
    delayCounter = 0;
  }

  if(!delayCounter) // закончили отсчёт задержки после включения питания
  {
#ifdef POWER_DEBUG    
    DEBUG_LOGLN(F("PWR: delay counter done!"));
#endif    
    bOnTimer = false;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::turnOn()
{
  
  // включаем питание
    out(true);
    
    // сбрасываем таймер выключения
    bOffTimer = false;

    // смотрим - если мы ещё не взвели таймер изменения задержки включения - взводим
    if(!bOnTimer)
    {
#ifdef POWER_DEBUG      
      DEBUG_LOG(F("PWR: turned ON, start delay counter: ")); 
#endif      
      bOnTimer = true;
      timer = millis();
      
      WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
      delayCounter = bnd.PowerOnDelay; // инициализируем полную задержку, поскольку не было потребителей

#ifdef POWER_DEBUG
      DEBUG_LOGLN(String(delayCounter));
#endif      
    }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t PowerManagerClass::PumpOn(uint8_t pump)
{
  WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return 0;
  }  

  // тут надо проверять - есть ли хотя бы один привязанный канал полива?
  if(!bnd.WateringChannels)
    return 0;

   turnOn(); // включаем питание

  if(pump == 0)
  {
    pump1On = true;
  }
  else
  {
    pump2On = true;
  }

   return delayCounter;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::PumpOff(uint8_t pump)
{
  WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет управления питанием
  {
    return;
  } 

  // тут надо проверять - есть ли хотя бы один привязанный канал полива?
  if(!bnd.WateringChannels)
    return;   
  
  if(pump == 0)
  {
    pump1On = false;
  }
  else
  {
    pump2On = false;
  }

   turnOff(); // выключаем питание  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t PowerManagerClass::DoorWantMove(uint8_t door)
{
#ifdef POWER_DEBUG  
  DEBUG_LOG(F("PWR: door want move, door #"));
  DEBUG_LOGLN(String(door));
#endif  
  
   WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
#ifdef POWER_DEBUG    
    DEBUG_LOGLN(F("PWR: no binding!"));
#endif    
    return 0;
  }  
    uint8_t doorBit = 1 << door; // номер бита двери, который мы контролируем
    
    if(!(bnd.DoorBinding & doorBit)) // это дверь не указана в привязке контролируемых по питанию, ей ждать задержки не надо
    {
#ifdef POWER_DEBUG      
      DEBUG_LOGLN(F("PWR: door is not our consumer!"));
#endif      
      return 0;
    }

    turnOn(); // включаем питание

    // запоминаем, что дверь движется
    doorsState |= doorBit;

    // переменная delayCounter будет обновляться автоматически, и следующая дверь, запросившае движение - 
    // получит уже меньшую паузу до начала движения
    return delayCounter;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::DoorMoveDone(uint8_t door)
{
  // дверь закончила движение
#ifdef POWER_DEBUG  
  DEBUG_LOG(F("PWR: door move done, door #"));
  DEBUG_LOGLN(String(door));
#endif  
  
  WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет управления питанием
  {
#ifdef POWER_DEBUG    
    DEBUG_LOGLN(F("PWR: no binding!"));
#endif    
    return;
  }  
    uint8_t doorBit = 1 << door; // номер бита двери, который мы контролируем
    
    if(!(bnd.DoorBinding & doorBit)) // эта дверь не указана в привязке контролируемых по питанию
    {
#ifdef POWER_DEBUG      
      DEBUG_LOGLN(F("PWR: door is not our consumer!"));
#endif      
      return;
    }

   // сбрасываем флаг движения двери
   doorsState &= ~doorBit;

   // теперь смотрим: если вообще никакие из контролируемых дверей - не движутся - надо выключать выход
   // и сбросить таймер

   turnOff(); // выключаем питание
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t PowerManagerClass::WindowWantMove(uint8_t window)
{
#ifdef POWER_DEBUG  
  DEBUG_LOG(F("PWR: window want move, window #"));
  DEBUG_LOGLN(String(window));
#endif  
  
   WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
#ifdef POWER_DEBUG    
    DEBUG_LOGLN(F("PWR: no binding!"));
#endif    
    return 0;
  }  
    uint16_t windowBit = 1 << window; // номер бита окна, который мы контролируем
    
    if(!(bnd.LinkedChannels & windowBit)) // это окно не указано в привязке контролируемых по питанию, ему ждать задержки не надо
    {
#ifdef POWER_DEBUG      
      DEBUG_LOGLN(F("PWR: window is not our consumer!"));
#endif      
      return 0;
    }

    turnOn(); // включаем питание

    // запоминаем, что окно движется
    windowsState |= windowBit;

    // переменная delayCounter будет обновляться автоматически, и следующее окно, запросившее движение - 
    // получит уже меньшую паузу до начала движения
    return delayCounter;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::WindowMoveDone(uint8_t window)
{
  // окно закончило движение
#ifdef POWER_DEBUG  
  DEBUG_LOG(F("PWR: window move done, window #"));
  DEBUG_LOGLN(String(window));
#endif  
  
  WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет управления питанием
  {
#ifdef POWER_DEBUG    
    DEBUG_LOGLN(F("PWR: no binding!"));
#endif    
    return;
  }  
    uint16_t windowBit = 1 << window; // номер бита окна, который мы контролируем
    
    if(!(bnd.LinkedChannels & windowBit)) // это окно не указано в привязке контролируемых по питанию
    {
#ifdef POWER_DEBUG      
      DEBUG_LOGLN(F("PWR: window is not our consumer!"));
#endif      
      return;
    }

   // сбрасываем флаг движения окна
   windowsState &= ~windowBit;

   // теперь смотрим: если вообще никакие из контролируемых окон - не движутся - надо выключать выход
   // и сбросить таймер

   turnOff(); // выключаем питание
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t PowerManagerClass::WateringOn(uint8_t channel)
{
   WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return 0;
  }  
    uint16_t channelBit = 1 << channel; // номер бита канала полива, который мы контролируем
    
    if(!(bnd.WateringChannels & channelBit)) // это канал полива не указан в привязке контролируемых по питанию, ему ждать задержки не надо
    {
      return 0;
    }

    turnOn(); // включаем питание

    // запоминаем, что канал полива включен
    wateringState |= channelBit;

    return delayCounter;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::WateringOff(uint8_t channel)
{
  // канал полива выключается
  
  WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  if(bnd.LinkType == linkUnbinded) // нет управления питанием
  {
    return;
  }  
    uint16_t channelBit = 1 << channel; // номер бита канала полива, который мы контролируем
    
    if(!(bnd.WateringChannels & channelBit)) // это канал полива не указан в привязке контролируемых по питанию
    {
      return;
    }

   // сбрасываем флаг активности канала полива
   wateringState &= ~channelBit;

   turnOff(); // выключаем питание
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PowerManagerClass::isNoConsumers()
{
  return (windowsState == 0) && (wateringState == 0) && !pump1On && !pump2On && (doorsState == 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PowerManagerClass::turnOff()
{
  if(isNoConsumers() && !bOffTimer && bOn) // нет активных потребителей питания
   {
#ifdef POWER_DEBUG    
      DEBUG_LOGLN(F("PWR: no consumers, turn OFF!"));
#endif      
      
      bOnTimer = false; // сбрасываем таймер включения, т.к. никто никуда не движется, выход будет выключен, следовательно - надо перед первым включением вернуть полную задержку

      // выставляем таймер выключения
      bOffTimer = true;
      offTimer = millis();
   }  
}
//--------------------------------------------------------------------------------------------------------------------------------------


